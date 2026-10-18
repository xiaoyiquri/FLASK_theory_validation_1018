"""First and final paths are summarized separately; failures never count as zero."""
import argparse
from collections import Counter
import csv
import math
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from experiment_io import load, resolve_file, save_json


def metric(values):
    values = [float(v) for v in values if v is not None and v != '']
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite metric')
    return dict(n=len(values), mean=statistics.mean(values) if values else None,
                sample_sd=statistics.stdev(values) if len(values)>1 else None,
                median=statistics.median(values) if values else None)


def display(value):
    if value['mean'] is None:
        return 'NA'
    sd=f"{value['sample_sd']:.3f}" if value['sample_sd'] is not None else 'NA'
    return f"{value['mean']:.3f} +/- {sd}"


def read_csv(path):
    with Path(path).open(encoding='utf-8', newline='') as source:
        return list(csv.DictReader(source))


def write_csv(path, rows):
    if not rows:
        return
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open('w', encoding='utf-8', newline='') as target:
        writer=csv.DictWriter(target, fieldnames=fields);writer.writeheader();writer.writerows(rows)


def summarize(root):
    root=Path(root)
    import json
    manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('result_version')!=2:
        raise ValueError('New measurements require version 2; old final-path cost/length are not first-path measurements')
    rows=read_csv(root/'runs.csv'); budgets=read_csv(root/'budget_metrics.csv')
    repeats=manifest['repeats']; expected_per_scene=len(manifest['seeds'])*repeats
    expected=len(manifest['scenarios'])*expected_per_scene
    if len(rows)!=expected:
        raise ValueError(f'Incomplete run: {len(rows)}/{expected}')
    keys={(r['scenario'],r['seed'],r['repeat']) for r in rows}
    if len(keys)!=len(rows):
        raise ValueError('Duplicate run identity')
    expected_budgets=expected*len(manifest['checkpoints_seconds'])
    if len(budgets)!=expected_budgets:
        raise ValueError(f'Incomplete checkpoint records: {len(budgets)}/{expected_budgets}')
    if len({(r['scenario'],r['seed'],r['repeat'],float(r['budget_seconds'])) for r in budgets})!=len(budgets):
        raise ValueError('Duplicate checkpoint identity')
    groups=[]; budget_stats=[]; diagnostics=[]; rejections=[]
    first_table=['| Environment | Success | First time (ms) | First J | First length (m) | First execution duration (s) |',
                 '| --- | --- | --- | --- | --- | --- |']
    final_table=['| Environment | Success | Final J | Final length (m) | Execution duration (s) | Planning incl. shortcut (ms) | Last improvement (ms) |',
                 '| --- | --- | --- | --- | --- | --- | --- |']
    for source in manifest['scenarios']:
        scene=Path(source).stem; records=[r for r in rows if r['scenario']==scene]
        if len(records)!=expected_per_scene:
            raise ValueError('Incomplete scenario '+scene)
        successes=[r for r in records if r['status']=='SUCCESS' and r['exit_code']=='0']
        group=dict(scenario=scene, runs=len(records), successes=len(successes),
                   success_rate=len(successes)/len(records), failure_reasons=dict(Counter(r['reason'] for r in records if r not in successes)),
                   native_first_status=dict(Counter(r['native_first_status'] for r in records)),
                   native_final_status=dict(Counter(r['native_final_status'] for r in records)))
        fields=['first_solution_ms','first_cost','first_length_m','first_duration','cost',
                'trajectory_length_m','duration','planning_ms','final_available_ms']
        for key in fields: group[key]=metric([r[key] for r in successes])
        group['failed_planning_ms']=metric([r['planning_ms'] for r in records if r not in successes])
        groups.append(group)
        label=scene.removeprefix('dynobench_'); count=f'{len(successes)}/{len(records)}'
        first_table.append('| '+' | '.join([label,count]+[display(group[k]) for k in fields[:4]])+' |')
        final_table.append('| '+' | '.join([label,count]+[display(group[k]) for k in fields[4:]])+' |')
        for t in manifest['checkpoints_seconds']:
            checkpoint=[r for r in budgets if r['scenario']==scene and float(r['budget_seconds'])==t]
            available=[r for r in checkpoint if r['has_solution'].lower()=='true']
            summary=dict(scenario=scene,budget_seconds=t,runs=len(checkpoint),successes=len(available),
                         success_rate=len(available)/len(checkpoint))
            for key in ['cost','trajectory_length_m','duration']:
                m=metric([r[key] for r in available])
                summary.update({key+'_'+stat:value for stat,value in m.items()})
            budget_stats.append(summary)
    for row in rows:
        source=root/row['trajectory_file']
        if not source.exists(): continue
        document=load(source)
        identity={k:row[k] for k in ['scenario','seed','repeat']}
        for key,value in document['validation']['failure_counts'].items():
            rejections.append(dict(identity,status=row['status'],termination_reason=row['reason'],reason=key,count=value,
                                   interval_rejections=document['validation']['interval_rejections']))
        if row['status']!='SUCCESS': continue
        for stage in ['first','final']:
            snapshot=load(resolve_file(root,row[stage+'_trajectory_file']))
            d=snapshot['diagnostics']; entry=dict(identity,stage=stage)
            entry.update({k:v for k,v in d.items() if k.startswith('max_')})
            for endpoint in ['start','goal']:
                entry.update({endpoint+'_'+k:v for k,v in d[endpoint].items()})
            diagnostics.append(entry)
    lines=['# FLASK-RRTConnect first / final / budget measurements','',
           f"Seeds: {manifest['seeds']}; repeats per seed: {repeats}; total planning budget: {manifest['budget_seconds']} s.",
           'J = rho*T + integral(w^T R w dt); execution duration is not planning time.',
           'Means +/- sample SD are over successful runs only. Before a first solution, cost and length are missing.',
           'First metrics belong to the saved raw RRTConnect path; final metrics belong to the shortcut path.',
           'Search and shortcut share the budget. Export, quadrature and native checks are outside the measured planning clock.','',
           '## First solution','']+first_table+['','## Final solution','']+final_table+['','## Checkpoints','',
           '| Environment | Budget (s) | Success | Mean J (available runs) | Mean length (m) |',
           '| --- | --- | --- | --- | --- |']
    for b in budget_stats:
        cost='NA' if b['cost_mean'] is None else f"{b['cost_mean']:.6g}"
        length='NA' if b['trajectory_length_m_mean'] is None else f"{b['trajectory_length_m_mean']:.6g}"
        lines.append(f"| {b['scenario']} | {b['budget_seconds']:g} | {b['successes']}/{b['runs']} | {cost} | {length} |")
    lines += ['', 'A changing available-run cohort can change the conditional mean; it is not an unconditional optimality claim.',
              'RRTConnect is not restarted after termination. Later checkpoints hold its final incumbent.',
              'Paper dynamics are retained; model_differences.json records differences from original DynoBench models.',
              'Reduced-r2 success is separate from native dynamics certification. See native_validation.json.','']
    for group in groups:
        if group['failure_reasons']:
            lines.append(f"- {group['scenario']}: {group['failure_reasons']}; failed planning time: {display(group['failed_planning_ms'])} ms")
    save_json(root/'statistics.json',dict(groups=groups, budget_statistics=budget_stats))
    write_csv(root/'budget_statistics.csv',budget_stats)
    write_csv(root/'diagnostics.csv',diagnostics);write_csv(root/'rejection_counts.csv',rejections)
    report='\n'.join(lines)+'\n';(root/'statistics.md').write_text(report,encoding='utf-8')
    (root/'summary.md').write_text(report,encoding='utf-8')
    # Analysis can be rerun without rerunning planning. Record the code/data
    # actually used here separately from the immutable run-start manifest.
    import hashlib
    tools=Path(__file__).resolve().parent
    save_json(root/'analysis_manifest.json',dict(
        source_sha256={name:hashlib.sha256((tools/name).read_bytes()).hexdigest()
                       for name in ['summarize_runs.py','experiment_io.py']},
        input_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest()
                      for name in ['manifest.json','runs.csv','budget_metrics.csv']}))
    print(report)
    return groups


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('results',type=Path)
    summarize(p.parse_args().results)


if __name__=='__main__':
    main()
