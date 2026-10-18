"""Shared version-2 measurement semantics. Times are planning wall seconds."""
import json
import math
from pathlib import Path

import yaml
SAFE_LOADER = getattr(yaml, 'CSafeLoader', yaml.SafeLoader)

CHECKPOINTS = [.001, .005, .01, .05, .1, .5, 1., 5., 10.]
MODEL_KEYS = ('dynamics', 'max_vel', 'min_vel', 'max_angular_vel', 'min_angular_vel',
              'm', 'I', 'l', 'J_v', 'motor_control', 'max_f', 'arm_length', 't2t',
              'u_lb', 'u_ub', 'dt', 'size', 'shape', 'drag_against_vel')


def load(path):
    with Path(path).open(encoding='utf-8') as source:
        return yaml.load(source, Loader=SAFE_LOADER)


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def resolve_file(root, name):
    path = Path(name)
    if path.is_absolute() and path.exists():
        return path
    candidate = Path(root)/path
    if candidate.exists():
        return candidate
    raise FileNotFoundError(candidate)


def model_differences(planning, native):
    def same(a, b):
        if isinstance(a, (int, float)) and isinstance(b, (int, float)):
            return math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-12)
        if isinstance(a, list) and isinstance(b, list):
            return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
        return a == b
    return {key: {'paper': planning.get(key), 'native': native.get(key)}
            for key in MODEL_KEYS if not same(planning.get(key), native.get(key))}


def configuration_snapshot(path):
    path = Path(path)
    config = load(path)
    if 'dynobench' not in config:
        return config
    d = config['dynobench']
    for key, snapshot in [('environment', 'dynobench_environment_snapshot'),
                          ('model', 'dynobench_model_snapshot'),
                          ('visual_model', 'dynobench_visual_model_snapshot')]:
        if key in d:
            source = Path(d[key])
            config[snapshot] = load(source if source.is_absolute() else path.parent/source)
    return config


def validate_history(document):
    if document.get('result_version') != 2:
        raise ValueError('Version-2 output required: old final trajectories cannot reconstruct a first-solution path')
    history = document.get('incumbents') or []
    previous_time, previous_cost = -1., math.inf
    budget = document['budget_seconds']
    for i, event in enumerate(history):
        t, cost = event['available_seconds'], event['cost']
        values = [t, cost, event['trajectory_length_m'], event['duration']]
        if not all(math.isfinite(x) and x >= 0 for x in values):
            raise ValueError('Invalid incumbent metric')
        if t < previous_time or t > budget or cost >= previous_cost:
            raise ValueError('Incumbents must be in time order, within budget and strictly cost improving')
        if event['index'] != i or (i == 0 and event['stage'] != 'first_solution'):
            raise ValueError('Invalid first-solution event')
        previous_time, previous_cost = t, cost
    if bool(history) != bool(document['success']):
        raise ValueError('Success/history mismatch')
    if history and not math.isclose(history[0]['available_seconds'], document['search_seconds'], abs_tol=1e-12):
        raise ValueError('First-solution time/path mismatch')
    return history


def incumbent_at(document, seconds):
    if not math.isfinite(seconds) or seconds < 0 or seconds > document['budget_seconds']+1e-12:
        raise ValueError('Checkpoint outside recorded budget')
    events = validate_history(document)
    candidates = [event for event in events if event['available_seconds'] <= seconds]
    return candidates[-1] if candidates else None


def checkpoints(budget, requested=None):
    values = CHECKPOINTS if requested is None else requested
    if not math.isfinite(budget) or budget <= 0 or any(not math.isfinite(t) or t <= 0 for t in values):
        raise ValueError('Budget and checkpoints must be positive and finite')
    if requested is not None and any(t > budget for t in values):
        raise ValueError('Requested checkpoint exceeds budget')
    return sorted(set([t for t in values if t <= budget]+[float(budget)]))


def checkpoint_rows(document, times, identity):
    out = []
    for seconds in times:
        event = incumbent_at(document, seconds)
        row = dict(identity, budget_seconds=seconds, has_solution=bool(event),
                   cost=None, trajectory_length_m=None, duration=None,
                   available_seconds=None, trajectory_file=None,
                   held_after_termination=bool(event and seconds > document['planning_seconds']))
        if event:
            row.update({key: event[key] for key in
                        ['cost', 'trajectory_length_m', 'duration', 'available_seconds', 'trajectory_file']})
        out.append(row)
    return out
