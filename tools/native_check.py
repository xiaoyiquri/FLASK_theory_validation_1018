"""Optional real DynoBench checker invocation; never substitutes a local checker."""
import subprocess
from pathlib import Path
import yaml

from experiment_io import load, model_differences


def check_stage(checker, models, config, trajectory, work, timeout=30):
    if not trajectory:
        return {'status': 'no_solution'}
    if checker is None:
        return {'status': 'not_requested', 'verified': False}
    checker = Path(checker)
    if not checker.is_file():
        return {'status': 'not_available', 'verified': False, 'reason': str(checker)}
    env = config.get('dynobench_environment_snapshot')
    if not env:
        return {'status': 'unsupported_environment', 'verified': False}
    model_file = Path(models)/(env['robots'][0]['type']+'.yaml')
    if not model_file.is_file():
        return {'status': 'model_missing', 'verified': False, 'reason': str(model_file)}
    differences = model_differences(config['dynobench_model_snapshot'], load(model_file))
    if differences:
        return {'status': 'model_mismatch', 'verified': False, 'differences': differences}
    native = Path(trajectory).with_suffix('.dynobench.yaml')
    if not native.is_file():
        return {'status': 'trajectory_missing', 'verified': False}
    work = Path(work); work.mkdir(parents=True, exist_ok=True)
    environment = work/'environment.yaml'
    environment.write_text(yaml.safe_dump(env, sort_keys=False), encoding='utf-8')
    command = [str(checker.resolve()), '--models_base_path', Path(models).resolve().as_posix()+'/',
               '--env_file', str(environment.resolve()), '--result_file', str(native.resolve())]
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        log = result.stdout+result.stderr
        # Source checker returns 0 AND prints an exact OK line on feasibility.
        passed = result.returncode == 0 and 'OK' in result.stdout.splitlines()
        report = {'status': 'passed' if passed else 'failed', 'verified': passed,
                  'exit_code': result.returncode, 'command': command}
    except subprocess.TimeoutExpired:
        log = 'Native checker process timeout\n'
        report = {'status': 'timeout', 'verified': False, 'command': command}
    except OSError as error:
        log = str(error)
        report = {'status': 'error', 'verified': False, 'command': command, 'reason': str(error)}
    (work/'checker.log').write_text(log, encoding='utf-8')
    report['log'] = str(work/'checker.log')
    return report
