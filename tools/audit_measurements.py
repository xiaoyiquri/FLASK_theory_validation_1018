"""Independently integrate saved flat polynomials and audit checkpoint provenance."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from experiment_io import incumbent_at,load,resolve_file,save_json,validate_history


def derivative(segment,order,times):
    coefficients=np.asarray(segment['power_coefficients'],dtype=float)
    d=np.polynomial.polynomial.polyder(coefficients,m=order,axis=1)
    normalized=segment['normalized_time'];T=segment['duration']
    x=np.asarray(times)/T if normalized else np.asarray(times)
    values=np.array([np.polynomial.polynomial.polyval(x,row) for row in d])
    return values/(T**order) if normalized else values


def audit(root):
    root=Path(root)
    with (root/'runs.csv').open() as f:rows=list(csv.DictReader(f))
    with (root/'budget_metrics.csv').open() as f:budgets=list(csv.DictReader(f))
    manifest=json.loads((root/'manifest.json').read_text())
    expected=len(manifest['scenarios'])*len(manifest['seeds'])*manifest['repeats']
    assert len(rows)==expected and len(budgets)==expected*len(manifest['checkpoints_seconds'])
    docs={};snapshots=0;max_cost_error=0.;max_length_error=0.;max_endpoint_error=0.
    g,w=np.polynomial.legendre.leggauss(8);lg,lw=np.polynomial.legendre.leggauss(128)
    for row in rows:
        doc=load(root/row['trajectory_file']);validate_history(doc)
        docs[(row['scenario'],row['seed'],row['repeat'])]=doc
        assert doc['success']==(row['status']=='SUCCESS')
        for event in doc['incumbents']:
            assert resolve_file(root,event['trajectory_file']).is_file()
        if not doc['success']:
            assert doc['cost'] is None and doc['first_solution_cost'] is None
            continue
        assert len(doc['incumbents'])==doc['shortcuts']+1
        assert doc['first_solution_cost']==doc['incumbents'][0]['cost']
        for stage in ['first_solution','final_solution']:
            trajectory=load(resolve_file(root,doc[stage]['trajectory_file']));snapshots+=1
            cfg=trajectory['input_configuration'];R=np.asarray(cfg['weight'],dtype=float)
            rho=cfg['rho'];order=trajectory['r'];energy=0.;length=0.;duration=0.
            previous=np.asarray(trajectory['flat_start'],dtype=float)
            dim=len(cfg['dynobench_environment_snapshot']['environment']['min'])
            for segment in trajectory['segments']:
                T=segment['duration'];duration+=T
                u=derivative(segment,order,(g+1)*T/2)
                energy+=T/2*np.dot(w,np.einsum('ik,ij,jk->k',u,R,u))
                velocity=derivative(segment,1,(lg+1)*T/2)[:dim]
                length+=T/2*np.dot(lw,np.linalg.norm(velocity,axis=0))
                start=np.concatenate([derivative(segment,d,0) for d in range(order)])
                end=np.concatenate([derivative(segment,d,T) for d in range(order)])
                max_endpoint_error=max(max_endpoint_error,float(np.max(np.abs(start-previous))))
                previous=end
            max_endpoint_error=max(max_endpoint_error,float(np.max(np.abs(previous-np.asarray(trajectory['flat_goal'])))))
            error=abs(energy+rho*duration-trajectory['cost'])/max(1.,trajectory['cost'])
            length_error=abs(length-trajectory['trajectory_length_m'])
            max_cost_error=max(max_cost_error,error);max_length_error=max(max_length_error,length_error)
            assert error<1e-7,(row['scenario'],stage,'objective',error)
            assert length_error<.002,(row['scenario'],stage,'length',length_error)
    assert max_endpoint_error<1e-6
    for b in budgets:
        doc=docs[(b['scenario'],b['seed'],b['repeat'])];event=incumbent_at(doc,float(b['budget_seconds']))
        assert bool(event)==(b['has_solution'].lower()=='true')
        for key in ['cost','trajectory_length_m','duration']:
            if event:assert math.isclose(float(b[key]),event[key],rel_tol=1e-12,abs_tol=1e-12)
            else:assert b[key]==''
    for name,digest in manifest['sources'].items():
        if name.endswith(('.h','.cpp')):
            assert hashlib.sha256((Path(__file__).resolve().parents[1]/name).read_bytes()).hexdigest()==digest
    native=json.loads((root/'native_validation.json').read_text())
    report=dict(runs=len(rows),successes=sum(r['status']=='SUCCESS' for r in rows),checkpoints=len(budgets),
                independently_integrated_first_final_paths=snapshots,max_objective_relative_error=max_cost_error,
                max_length_difference_m=max_length_error,max_flat_endpoint_or_join_error=max_endpoint_error,
                native_verified_stages=sum(bool(r.get('verified')) for r in native),
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    save_json(root/'measurement_audit.json',report);print(json.dumps(report,indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('results',type=Path)
    audit(p.parse_args().results)
