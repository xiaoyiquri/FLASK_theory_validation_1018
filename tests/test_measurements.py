"""Synthetic fixtures test measurement semantics, not planner performance."""
import copy
import csv
import json
import io
from contextlib import redirect_stdout
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import yaml
from experiment_io import checkpoints, checkpoint_rows, configuration_snapshot, incumbent_at, model_differences, validate_history
from native_check import check_stage
from plot_six import rotation_from_xyzw
from summarize_runs import summarize


def document():
    return {'result_version':2,'success':True,'budget_seconds':10,'planning_seconds':.04,'search_seconds':.002,
            'incumbents':[{'index':0,'stage':'first_solution','available_seconds':.002,'cost':7.,
                           'trajectory_length_m':4.,'duration':3.,'trajectory_file':'first_solution/a.yaml'},
                          {'index':1,'stage':'shortcut','available_seconds':.02,'cost':5.,
                           'trajectory_length_m':4.5,'duration':2.,'trajectory_file':'incumbents/a_1.yaml'}]}


class Measurements(unittest.TestCase):
    def test_failed_runs_keep_rejection_counts_and_checkpoint_denominator(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);scene='dynobench_uni_bugtrap'
            manifest={'result_version':2,'seeds':[20],'repeats':1,'budget_seconds':10,
                      'checkpoints_seconds':[.001,10],'scenarios':{'/examples/'+scene+'.yaml':'fixture'}}
            (folder/'manifest.json').write_text(json.dumps(manifest))
            d=document();d.update(success=False,incumbents=[],planning_seconds=10,search_seconds=10,
                                  validation={'failure_counts':{'omega':17},'interval_rejections':3})
            (folder/'failed.yaml').write_text(yaml.safe_dump(d))
            row={'scenario':scene,'seed':20,'repeat':1,'status':'FAILED','exit_code':2,'reason':'timeout',
                 'planning_ms':10000,'trajectory_file':'failed.yaml','native_first_status':'no_solution','native_final_status':'no_solution'}
            with (folder/'runs.csv').open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(row));writer.writeheader();writer.writerow(row)
            budgets=checkpoint_rows(d,[.001,10],{'scenario':scene,'seed':20,'repeat':1})
            with (folder/'budget_metrics.csv').open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(budgets[0]));writer.writeheader();writer.writerows(budgets)
            with redirect_stdout(io.StringIO()): groups=summarize(folder)
            self.assertEqual(groups[0]['successes'],0)
            self.assertIsNone(groups[0]['first_cost']['mean'])
            with (folder/'rejection_counts.csv').open() as f: counts=list(csv.DictReader(f))
            self.assertEqual(counts[0]['count'],'17');self.assertEqual(counts[0]['status'],'FAILED')

    def test_environment_and_original_model_are_snapshots_of_files(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder)
            env={'environment':{'min':[0,0],'max':[5,6],'obstacles':[{'type':'box','center':[2,3],'size':[.2,1.4]}]},
                 'robots':[{'type':'unicycle1_v0','start':[1,2,0],'goal':[4,2,0]}]}
            native={'size':[.5,.25],'max_vel':.5};paper={'size':[.5,.25],'max_vel':1}
            for name,value in [('env.yaml',env),('native.yaml',native),('paper.yaml',paper)]:
                (folder/name).write_text(yaml.safe_dump(value))
            scene={'robot':'unicycle','dynobench':{'environment':'env.yaml','model':'paper.yaml','visual_model':'native.yaml'}}
            path=folder/'scene.yaml';path.write_text(yaml.safe_dump(scene))
            saved=configuration_snapshot(path)
            (folder/'env.yaml').write_text('environment: {}')
            self.assertEqual(saved['dynobench_environment_snapshot'],env)
            self.assertEqual(saved['dynobench_visual_model_snapshot'],native)
            self.assertEqual(saved['dynobench_model_snapshot'],paper)

    def test_checkpoints_do_not_use_future_paths(self):
        d=document()
        self.assertIsNone(incumbent_at(d,.001))
        self.assertEqual(incumbent_at(d,.002)['cost'],7)
        self.assertEqual(incumbent_at(d,.01)['trajectory_file'],'first_solution/a.yaml')
        self.assertEqual(incumbent_at(d,.02)['cost'],5)
        self.assertEqual(incumbent_at(d,10)['trajectory_length_m'],4.5)

    def test_early_stop_holds_real_incumbent(self):
        rows=checkpoint_rows(document(),[.001,.005,1,10],{'seed':20})
        self.assertFalse(rows[0]['has_solution']);self.assertIsNone(rows[0]['cost'])
        self.assertFalse(rows[1]['held_after_termination'])
        self.assertTrue(rows[2]['held_after_termination'])
        self.assertEqual(rows[2]['cost'],rows[3]['cost'])

    def test_failure_stays_missing(self):
        d=document();d.update(success=False,incumbents=[])
        self.assertEqual(validate_history(d),[])
        self.assertTrue(all(r['cost'] is None for r in checkpoint_rows(d,[.01,10],{})))

    def test_late_and_nonmonotonic_histories_rejected(self):
        for change in [{'cost':8},{'available_seconds':11},{'available_seconds':.001},{'index':4}]:
            d=document();d['incumbents'][1].update(change)
            with self.assertRaises(ValueError):validate_history(d)
        with self.assertRaises(ValueError):validate_history({'result_version':1})
        self.assertEqual(checkpoints(10),[.001,.005,.01,.05,.1,.5,1.,5.,10.])
        with self.assertRaises(ValueError):checkpoints(1,[2])

    def test_model_profile_difference_is_not_geometry_guess(self):
        self.assertEqual(model_differences({'m':1.,'J_v':[1,2,3]},{'m':1,'J_v':[1.,2.,3.]}),{})
        differences=model_differences({'m':1,'motor_control':False},{'m':.034,'motor_control':True})
        self.assertEqual(set(differences),{'m','motor_control'})

    def test_quaternion_drawing_matches_xyzw_convention(self):
        import numpy as np
        q=np.array([0,0,2**-.5,2**-.5])
        expected=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        np.testing.assert_allclose(rotation_from_xyzw(q),expected,atol=1e-14)
        np.testing.assert_allclose(rotation_from_xyzw(-q),expected,atol=1e-14)

    def test_native_adapter_uses_real_command_and_separates_mismatch(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);checker=folder/'checker';checker.write_text('test placeholder')
            model={'dynamics':'quad2d','m':.034};(folder/'quad2d_v0.yaml').write_text(yaml.safe_dump(model))
            trajectory=folder/'path.yaml';trajectory.with_suffix('.dynobench.yaml').write_text('result: []')
            config={'dynobench_environment_snapshot':{'robots':[{'type':'quad2d_v0'}]},'dynobench_model_snapshot':model}
            completed=subprocess.CompletedProcess([],0,'OK\n','')
            with patch('native_check.subprocess.run',return_value=completed) as run:
                result=check_stage(checker,folder,config,trajectory,folder/'work')
                self.assertEqual(result['status'],'passed')
                command=run.call_args.args[0]
                self.assertIn('--result_file',command)
                self.assertTrue(command[command.index('--models_base_path')+1].endswith('/'))
            config['dynobench_model_snapshot']={'dynamics':'quad2d','m':1}
            with patch('native_check.subprocess.run') as run:
                self.assertEqual(check_stage(checker,folder,config,trajectory,folder/'other')['status'],'model_mismatch')
                run.assert_not_called()
            self.assertEqual(check_stage(None,folder,config,trajectory,folder)['status'],'not_requested')


if __name__=='__main__':
    unittest.main()
