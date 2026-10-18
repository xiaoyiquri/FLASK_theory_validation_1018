"""Regenerate the explicit local FLASK-RRTConnect profiles, not author defaults."""
import json
from pathlib import Path


def main():
    examples = Path(__file__).resolve().parents[1] / 'examples'
    for file in sorted(examples.glob('dynobench_*.yaml')):
        config = json.loads(file.read_text(encoding='utf-8'))
        config['r'] = 2
        config['rho'] = 1
        n = config['n']
        for side in ('lower', 'upper'):
            config['bounds'][side] = config['bounds'][side][:2*n]
        quad = config['robot'].startswith('quad')
        native_name = ('unicycle1_v2.yaml' if 'uni_wall' in file.stem else
                       'unicycle1_v0.yaml' if config['robot']=='unicycle' else
                       'quad2d_v0.yaml' if config['robot']=='quad2d' else 'quad3d_v0.yaml')
        config['dynobench']['visual_model'] = 'dynobench_original_models/'+native_name
        config['dynobench']['model_profile'] = 'paper; original native model retained for visualization/comparison'
        config['recording'] = {'checkpoints_seconds': [.001,.005,.01,.05,.1,.5,1,5,10]}
        if quad:
            config.pop('lifting', None)
        config['planner'].pop('unicycle_sample_speed', None)
        config['weight'] = [[int(i==j) for j in range(n)] for i in range(n)]
        config['planner'].update(type='rrtconnect', iterations=1000000, max_nodes=20000,
                                 timeout=10, range=.25, shortcut=True,
                                 shortcut_attempts=100, goal_bias=0, connect_steps=0)
        config['validation'].update(step=.01, continuous_sphere_collision=False,
                                   enforce_physical_continuity=not quad,
                                   enforce_physical_endpoints=not quad)
        config['parameter_provenance'] = {
            'r': 'paper practical r2; not the full-state Assumption 1 contract for quads',
            'rho': 'paper VII-A: 1', 'weight': 'user-specified identity; private experiment R unconfirmed',
            'sampler': 'uniform flat box; public OMPL reference, private VAMP experiment unconfirmed',
            'range_bounds_radius_resolution_budget': 'explicit local settings; paper defaults unconfirmed',
            'goal_bias_connect': 'public RRTConnect: no goal bias, connect until trapped/reached or resource limit',
            'validation': 'Algorithm 3 finite grid; extra continuous sphere certificate disabled'}
        file.write_text(json.dumps(config, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
