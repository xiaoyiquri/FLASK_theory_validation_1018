"""Six-environment measurements: first path, shortcut events and in-budget incumbents."""
import argparse
import csv
import hashlib
import json
import math
import platform
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
import yaml
from experiment_io import (CHECKPOINTS, checkpoints, checkpoint_rows, configuration_snapshot,
                           load, model_differences, save_json, validate_history)
from native_check import check_stage

SIX = ['uni_bugtrap', 'uni_wall', 'quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window']


def write_csv(path, rows, empty_fields=None):
    if not rows and empty_fields is None:
        return
    fields = list(dict.fromkeys(key for row in rows for key in row)) if rows else empty_fields
    with Path(path).open('w', newline='', encoding='utf-8') as target:
        writer = csv.DictWriter(target, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)


def main():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--executable', type=Path, required=True)
    p.add_argument('--scenarios', nargs='+', type=Path,
                   default=[root/'examples'/f'dynobench_{s}.yaml' for s in SIX])
    p.add_argument('--seeds', nargs='+', type=int, default=list(range(20, 40)))
    p.add_argument('--repeats', type=int, default=1)
    p.add_argument('--budget', type=float, default=10.)
    p.add_argument('--backend', choices=['scalar','simd'],default='simd')
    p.add_argument('--checkpoints', nargs='+', type=float)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--process-timeout', type=float)
    p.add_argument('--native-checker', type=Path)
    p.add_argument('--native-models', type=Path, default=root/'examples/dynobench_original_models')
    args = p.parse_args()
    try:
        times = checkpoints(args.budget, args.checkpoints)
    except ValueError as error:
        p.error(str(error))
    process_limit = args.process_timeout or args.budget+35
    if not math.isfinite(process_limit) or process_limit <= 0:
        p.error('process-timeout must be positive and finite')
    if not args.seeds or any(s < 0 or s > 2**32-1 for s in args.seeds) or len(set(args.seeds)) != len(args.seeds):
        p.error('seeds must be distinct uint32 integers')
    if args.repeats < 1:
        p.error('repeats must be positive')
    executable = args.executable.resolve(strict=True)
    scenarios = [s.resolve(strict=True) for s in args.scenarios]
    if len({s.stem for s in scenarios}) != len(scenarios):
        p.error('scenario names must be distinct')
    snapshots = {s.stem: configuration_snapshot(s) for s in scenarios}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'configurations').mkdir()
    for scene, config in snapshots.items():
        (args.output/'configurations'/f'{scene}.yaml').write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()
    sources = sorted(list((root/'include/flask').glob('*.h'))+list((root/'src').glob('*.cpp'))+list((root/'tools').glob('*.py'))+
                     [root/'CMakeLists.txt',root/'vendor/rrtpff/SOURCE_MANIFEST.json'])
    inputs = {}
    for path in scenarios:
        inputs[str(path)] = digest(path)
        for key in ['environment', 'model', 'visual_model']:
            value = snapshots[path.stem].get('dynobench', {}).get(key)
            if value:
                source = Path(value); source = source if source.is_absolute() else path.parent/source
                inputs[str(source.resolve())] = digest(source)
    manifest = dict(result_version=2, executable=str(executable), executable_sha256=digest(executable),
                    seeds=args.seeds, repeats=args.repeats, budget_seconds=args.budget,requested_backend=args.backend,
                    checkpoints_seconds=times, process_timeout=process_limit,
                    python=platform.python_version(), platform=platform.platform(),
                    scenarios={str(s): digest(s) for s in scenarios}, input_files=inputs,
                    sources={str(s.relative_to(root)): digest(s) for s in sources})
    manifest['native_checker']={'path':str(args.native_checker) if args.native_checker else None,
                                'sha256':digest(args.native_checker) if args.native_checker and args.native_checker.is_file() else None,
                                'models_directory':str(args.native_models.resolve())}
    save_json(args.output/'manifest.json', manifest)
    rows, events, budgets, certification, differences = [], [], [], [], {}
    for repeat in range(1, args.repeats+1):
        for scene in scenarios:
            config = snapshots[scene.stem]
            original = config.get('dynobench_visual_model_snapshot')
            differences[scene.stem] = model_differences(config.get('dynobench_model_snapshot', {}), original) if original else {'status': 'original_model_not_supplied'}
            for seed in args.seeds:
                suffix = f'_r{repeat:02d}' if args.repeats > 1 else ''
                name = f'{scene.stem}_{seed}{suffix}'
                result = args.output/f'{name}.yaml'
                identity = dict(scenario=scene.stem, seed=seed, repeat=repeat)
                row = dict(identity, status='ERROR', reason='', exit_code='', nodes='', bvp_queries='',
                           extension_attempts='', valid_extensions='', valid_extension_rate='', validation_backend='', budget_seconds=args.budget,
                           first_solution_ms='', first_cost='', first_length_m='', first_duration='',
                           first_trajectory_file='', final_available_ms='', planning_ms='', planning_seconds='',
                           search_seconds='', cost='', trajectory_length_m='', duration='', final_trajectory_file='',
                           process_seconds='', trajectory_file=result.name,
                           configuration_file=f'configurations/{scene.stem}.yaml', native_first_status='', native_final_status='')
                begin = time.monotonic(); document = None; run_events=[]; run_budgets=[]
                try:
                    run = subprocess.run([str(executable), str(scene), str(result.resolve()), str(seed), str(args.budget),args.backend],
                                         capture_output=True, text=True, timeout=process_limit,
                                         creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                    log = run.stdout+run.stderr; row['exit_code'] = run.returncode
                    if result.exists():
                        document = load(result); history = validate_history(document)
                        row.update(status='SUCCESS' if document['success'] and run.returncode == 0 else 'FAILED',
                                   reason=document['reason'], nodes=document['nodes'], bvp_queries=document['bvp_queries'],
                                   extension_attempts=document['extension_attempts'], valid_extensions=document['valid_extensions'],
                                   valid_extension_rate=document['valid_extension_rate'], validation_backend=document['validation_backend'],
                                   planning_seconds=document['planning_seconds'], planning_ms=1000*document['planning_seconds'],
                                   search_seconds=document['search_seconds'])
                        if run.returncode not in [0, 2]:
                            row.update(status='ERROR', reason='cli_error_after_result')
                        if history and row['status'] == 'SUCCESS':
                            first, final = history[0], document['final_solution']
                            row.update(first_solution_ms=1000*first['available_seconds'], first_cost=first['cost'],
                                       first_length_m=first['trajectory_length_m'], first_duration=first['duration'],
                                       first_trajectory_file=first['trajectory_file'], final_available_ms=1000*final['available_seconds'],
                                       cost=final['cost'], trajectory_length_m=final['trajectory_length_m'],
                                       duration=final['duration'], final_trajectory_file=final['trajectory_file'])
                        for event in history:
                            run_events.append(dict(identity, **event))
                        run_budgets=checkpoint_rows(document, times, identity)
                    else:
                        message = next((line for line in log.splitlines() if line.startswith('ERROR:')), 'cli_error')
                        row['reason'] = message
                except subprocess.TimeoutExpired as error:
                    row.update(status='PROCESS_TIMEOUT', reason='process_limit')
                    def decode(value):
                        return value.decode(errors='replace') if isinstance(value, bytes) else value or ''
                    log = decode(error.stdout)+decode(error.stderr)
                except OSError as error:
                    row.update(status='ERROR',reason='process_error: '+str(error));log=str(error)
                except (ValueError, KeyError, yaml.YAMLError) as error:
                    row.update(status='ERROR', reason=f'result_parse_error: {error}')
                    document=None; run_events=[]; run_budgets=[]
                    row['first_trajectory_file']=''; row['final_trajectory_file']=''
                    log = locals().get('log', '')+'\n'+str(error)
                row['process_seconds'] = time.monotonic()-begin
                (args.output/f'{name}.log').write_text(log, encoding='utf-8')
                # Errors/timeouts count in every checkpoint denominator and never
                # become zero-cost or unobserved successful samples.
                if document is None:
                    for t in times:
                        run_budgets.append(dict(identity, budget_seconds=t, has_solution=False, cost=None,
                                            trajectory_length_m=None, duration=None, available_seconds=None,
                                            trajectory_file=None, held_after_termination=False))
                events.extend(run_events); budgets.extend(run_budgets)
                for stage, key in [('first', 'first_trajectory_file'), ('final', 'final_trajectory_file')]:
                    path = args.output/row[key] if row[key] else None
                    report = check_stage(args.native_checker, args.native_models, config, path,
                                         args.output/'native_checks'/name/stage)
                    row[f'native_{stage}_status'] = report['status']
                    certification.append(dict(identity, stage=stage, **report))
                rows.append(row)
                print(f'round {repeat}/{args.repeats}', scene.stem, seed, row['status'], row['reason'], flush=True)
                write_csv(args.output/'runs.csv', rows)
                write_csv(args.output/'cost_time_events.csv', events,
                          ['scenario','seed','repeat','index','stage','available_seconds','cost','trajectory_length_m','duration','trajectory_file'])
                write_csv(args.output/'budget_metrics.csv', budgets)
                save_json(args.output/'native_validation.json', certification)
    save_json(args.output/'model_differences.json', differences)
    from summarize_runs import summarize
    summarize(args.output)
    return 0 if all(r['status'] == 'SUCCESS' for r in rows) else 2


if __name__ == '__main__':
    raise SystemExit(main())
