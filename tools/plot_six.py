"""DynoBench-sourced robot trajectories and observed cost-versus-planning-time plots."""
import argparse
import csv
import json
from pathlib import Path
import sys
import textwrap

# Deliberate project import, also when Python -I disables the script directory.
sys.path.insert(0,str(Path(__file__).resolve().parent))
from experiment_io import incumbent_at, load, resolve_file


def box_vertices(center,size,rotation=None):
    import numpy as np
    center=np.asarray(center); size=np.asarray(size)
    offsets=np.array([[x,y,z] for x in [-.5,.5] for y in [-.5,.5] for z in [-.5,.5]])*size
    return offsets@(np.eye(3) if rotation is None else rotation).T+center


def rotation_from_xyzw(q):
    import numpy as np
    x,y,z,w=np.asarray(q,dtype=float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def draw_box3(ax,vertices,color,alpha):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    faces=[[vertices[j] for j in f] for f in [(0,1,3,2),(4,5,7,6),(0,1,5,4),(2,3,7,6),(0,2,6,4),(1,3,7,5)]]
    ax.add_collection3d(Poly3DCollection(faces,facecolor=color,edgecolor=color,alpha=alpha,linewidths=.5))


def draw_robot(ax,state,robot,model,color,alpha=.65):
    import numpy as np
    from matplotlib.patches import Polygon,Circle
    p=np.asarray(state,dtype=float)
    size=np.asarray(model['size'],dtype=float)
    if robot in ['unicycle','quad2d']:
        theta=p[2]; c,s=np.cos(theta),np.sin(theta); rotation=np.array([[c,-s],[s,c]])
        corners=np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]])*size
        ax.add_patch(Polygon(corners@rotation.T+p[:2],closed=True,facecolor=color,edgecolor=color,alpha=alpha*.3,linewidth=1))
        if robot=='unicycle':
            end=p[:2]+rotation@np.array([size[0]*.48,0])
            ax.annotate('',xy=end,xytext=p[:2],arrowprops=dict(arrowstyle='->',color=color,lw=1,alpha=alpha))
        else:
            arm=model['l']; ends=np.array([[-arm,0],[arm,0]])@rotation.T+p[:2]
            ax.plot(ends[:,0],ends[:,1],color=color,alpha=alpha,lw=1.4)
            for end in ends:
                ax.add_patch(Circle(end,min(size[1]/4,arm/3),facecolor=color,alpha=alpha))
    elif robot=='quad3d':
        r=rotation_from_xyzw(p[3:7]);arm=model['arm_length'];a=arm/2**.5
        motors=np.array([[-a,-a,0],[-a,a,0],[a,a,0],[a,-a,0]])@r.T+p[:3]
        for i,j in [(0,2),(1,3)]:
            ax.plot(*motors[[i,j]].T,color=color,alpha=alpha,lw=1.5)
        ax.scatter(*motors.T,color=color,alpha=alpha,s=5)
        # Original native collision size is displayed, not the planner radius.
        if len(size)==1:
            angles=np.linspace(0,2*np.pi,40)
            for plane in [(0,1),(0,2)]:
                ring=np.zeros((40,3));ring[:,plane[0]]=size[0]*np.cos(angles);ring[:,plane[1]]=size[0]*np.sin(angles)
                ring=ring@r.T+p[:3];ax.plot(*ring.T,color=color,alpha=alpha*.25,lw=.5)
            scale=size[0]
        else:
            draw_box3(ax,box_vertices(p[:3],size,r),color,alpha*.12);scale=max(size)/2
        axis=r[:,2]*scale
        ax.quiver(*p[:3],*axis,color=color,alpha=alpha,arrow_length_ratio=.25,linewidth=.8)


def draw_environment(ax,config):
    import numpy as np
    from matplotlib.patches import Rectangle,Circle
    document=config['dynobench_environment_snapshot'];env=document['environment']
    dim=len(env['min'])
    for obstacle in env['obstacles']:
        c=np.array(obstacle['center']);kind=obstacle['type']
        if kind=='box':
            size=np.array(obstacle['size'])
            if dim==2: ax.add_patch(Rectangle(c-size/2,*size,facecolor='.48',edgecolor='.3',alpha=.7))
            else: draw_box3(ax,box_vertices(c,size),'.42',.22)
        elif kind=='sphere':
            radius=obstacle['radius']
            if dim==2: ax.add_patch(Circle(c,radius,color='.45',alpha=.65))
            else:
                u,v=np.mgrid[0:2*np.pi:20j,0:np.pi:12j]
                ax.plot_surface(c[0]+radius*np.cos(u)*np.sin(v),c[1]+radius*np.sin(u)*np.sin(v),c[2]+radius*np.cos(v),color='.5',alpha=.2)
        else: raise ValueError('Unsupported native obstacle '+kind)
    ax.set_xlim(env['min'][0],env['max'][0]);ax.set_ylim(env['min'][1],env['max'][1])
    ax.set_xlabel('x (m)');ax.set_ylabel('z (m)' if config['robot']=='quad2d' else 'y (m)')
    if dim==3:
        ax.set_zlim(env['min'][2],env['max'][2]);ax.set_zlabel('z (m)')
        ax.set_box_aspect(np.array(env['max'])-env['min']);ax.view_init(elev=23,azim=-62)
    else: ax.set_aspect('equal',adjustable='box')
    ax.grid(alpha=.15)
    return document['robots'][0]


def draw_path(ax,config,snapshot,first_ms,label,detail=True):
    import numpy as np
    from matplotlib.lines import Line2D
    native=config.get('dynobench_visual_model_snapshot')
    if not native:
        raise ValueError('Original DynoBench model snapshot missing; regenerate configuration and rerun')
    robot=config['robot'];problem=draw_environment(ax,config)
    dim=3 if robot=='quad3d' else 2
    for key,color in [('start','#19834b'),('goal','#d23b3b')]:
        state=problem[key]
        draw_robot(ax,state,robot,native,color,alpha=.95)
        ax.scatter(*state[:dim],color=color,s=25,zorder=20)
    if snapshot:
        states=np.array([s['state'] for s in snapshot['samples']])
        if len(states):
            ax.plot(*states[:,:dim].T,color='#176eab',linewidth=1.5,alpha=.95)
            indices=np.unique(np.linspace(0,len(states)-1,7,dtype=int))
            for i in indices: draw_robot(ax,states[i],robot,native,'#176eab',alpha=.6)
        text=(f"cost J={snapshot['cost']:.4g}   length={snapshot['trajectory_length_m']:.3f} m\n"
              f"first={first_ms:.3f} ms   available={1000*snapshot['available_seconds']:.3f} ms")
    else: text='J=NA   L=NA   first=NA\nNo solution available'
    ax.set_title(label,fontsize=10 if not detail else 12)
    if dim==3: ax.text2D(.02,.98,text,transform=ax.transAxes,va='top',fontsize=8,bbox=dict(facecolor='white',alpha=.8,edgecolor='none'))
    else: ax.text(.02,.98,text,transform=ax.transAxes,va='top',fontsize=8,bbox=dict(facecolor='white',alpha=.8,edgecolor='none'))
    if detail:
        ax.legend(handles=[Line2D([0],[0],color='#176eab',label='Trajectory / model poses'),
                           Line2D([0],[0],marker='o',color='#19834b',linestyle='',label='DynoBench start'),
                           Line2D([0],[0],marker='o',color='#d23b3b',linestyle='',label='DynoBench goal')],loc='lower left',fontsize=7)


def draw_cost_time(ax,records,root,manifest,scene,legend=True):
    import numpy as np
    from experiment_io import incumbent_at
    budget=manifest['budget_seconds'];documents=[];positive=[]
    for row in records:
        if row['status']!='SUCCESS': continue
        source=root/row['trajectory_file']
        if not source.exists(): continue
        doc=load(source);documents.append(doc)
        events=doc.get('incumbents') or []
        if not events: continue
        times=[max(e['available_seconds'],1e-9) for e in events]
        positive.extend(times)
        times.append(budget);costs=[e['cost'] for e in events]+[events[-1]['cost']]
        ax.step(times,costs,where='post',color='#176eab',alpha=.28,lw=.9)
    # Aggregate changes at actual discovery/improvement times, not just
    # checkpoint bins, so the success curve does not lag a first solution.
    times=sorted(set(manifest['checkpoints_seconds']+positive+[budget]));means=[];rates=[]
    for t in times:
        available=[e for doc in documents if (e:=incumbent_at(doc,t)) is not None]
        means.append(np.mean([e['cost'] for e in available]) if available else np.nan)
        rates.append(len(available)/len(records))
    ax.step(times,means,where='post',color='#b53630',lw=1.2,label='Mean J of available runs')
    right=ax.twinx();right.step(times,rates,where='post',color='#26824a',linestyle=':',label='Success fraction')
    right.set_ylim(0,1.04);right.set_ylabel('Success fraction',fontsize=8)
    ax.set_xscale('log');ax.set_xlim(min([times[0]]+positive)*.9,budget*1.04)
    ax.set_xlabel('Planning elapsed time (s)');ax.set_ylabel('LQMT cost J');ax.set_title(scene.removeprefix('dynobench_'))
    ax.grid(alpha=.2,which='both')
    if not any(e.get('incumbents') for e in documents):
        ax.text(.5,.5,'No observed solution',ha='center',transform=ax.transAxes)
    if legend:
        lines,labels=ax.get_legend_handles_labels();other,names=right.get_legend_handles_labels()
        ax.legend(lines+other,labels+names,fontsize=7,loc='best')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('results',type=Path);p.add_argument('--seed',type=int)
    p.add_argument('--repeat',type=int,default=1)
    p.add_argument('--stages',nargs='*',choices=['first','final'],default=['first','final'])
    p.add_argument('--at-budget',nargs='*',type=float,default=[])
    args=p.parse_args();root=args.results
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('result_version')!=2: raise ValueError('Version-2 experiment required for saved first paths and cost-time events')
    with (root/'runs.csv').open(encoding='utf-8') as source: rows=list(csv.DictReader(source))
    seed=manifest['seeds'][0] if args.seed is None else args.seed
    selected=[r for r in rows if int(r['seed'])==seed and int(r['repeat'])==args.repeat]
    if not selected: p.error('No records for requested seed/repeat')
    if any(t<=0 or t>manifest['budget_seconds'] for t in args.at_budget): p.error('Plot checkpoint outside budget')
    folder=root/'plots'/f'seed_{seed}_r{args.repeat:02d}';folder.mkdir(parents=True,exist_ok=True)
    stages=args.stages+[f'budget_{t:g}s' for t in args.at_budget]
    for stage in stages:
        overview=plt.figure(figsize=(17,10),layout='constrained')
        for i,row in enumerate(selected):
            if i>=6: raise ValueError('Six-scene overview accepts at most six different scenarios')
            config=load(resolve_file(root,row['configuration_file']))
            result=root/row['trajectory_file']
            doc=load(result) if result.exists() and row['status']=='SUCCESS' else None
            first_ms=float(row['first_solution_ms']) if row['first_solution_ms'] else None
            event=None;path=None
            if doc:
                if stage=='first': event=doc.get('first_solution')
                elif stage=='final': event=doc.get('final_solution')
                else: event=incumbent_at(doc,float(stage[len('budget_'):-1]))
            if event: path=load(resolve_file(root,event['trajectory_file']))
            label=row['scenario'].removeprefix('dynobench_')+' | '+stage
            projection='3d' if config['robot']=='quad3d' else None
            ax=overview.add_subplot(2,3,i+1,projection=projection)
            draw_path(ax,config,path,first_ms,label,detail=False)
            fig=plt.figure(figsize=(9,7),layout='constrained');ax=fig.add_subplot(111,projection=projection)
            draw_path(ax,config,path,first_ms,label)
            if not path:
                message='No incumbent by this checkpoint' if stage.startswith('budget_') else row['reason']
                ax.set_title(label+'\n'+textwrap.fill(message,65),fontsize=11)
            fig.suptitle('Paper dynamics | original DynoBench geometry / endpoints | reduced-r2 continuity: diagnostic',fontsize=8)
            fig.savefig(folder/f"{row['scenario']}_{stage}.png",dpi=170);plt.close(fig)
        overview.suptitle(f'FLASK-RRTConnect | seed={seed} | {stage} | native geometry; paper dynamics')
        overview.savefig(root/f'paths_{stage}.png',dpi=170)
        if stage=='final': overview.savefig(root/'paths.png',dpi=170)
        plt.close(overview)
    cost_overview=plt.figure(figsize=(17,10),layout='constrained')
    for i,source in enumerate(manifest['scenarios']):
        scene=Path(source).stem;records=[r for r in rows if r['scenario']==scene]
        ax=cost_overview.add_subplot(2,3,i+1);draw_cost_time(ax,records,root,manifest,scene,False)
        fig,ax=plt.subplots(figsize=(9,6),layout='constrained');draw_cost_time(ax,records,root,manifest,scene)
        fig.suptitle('Thin lines: individual incumbents | conditional mean has a changing cohort | final held after stop',fontsize=8)
        fig.savefig(root/f'{scene}_cost_time.png',dpi=180);plt.close(fig)
    cost_overview.suptitle('Observed incumbent cost vs planning time | all seeds | conditional mean + success fraction')
    cost_overview.savefig(root/'cost_time.png',dpi=180);plt.close(cost_overview)
    import hashlib
    from experiment_io import save_json
    tools=Path(__file__).resolve().parent
    inputs=['manifest.json','runs.csv']+[r['trajectory_file'] for r in rows if (root/r['trajectory_file']).exists()]
    for row in selected:
        inputs.append(row['configuration_file'])
        inputs.extend(row[k] for k in ['first_trajectory_file','final_trajectory_file'] if row[k])
    save_json(root/'plot_manifest.json',dict(seed=seed,repeat=args.repeat,stages=stages,checkpoints=args.at_budget,
        source_sha256={name:hashlib.sha256((tools/name).read_bytes()).hexdigest() for name in ['plot_six.py','experiment_io.py']},
        input_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in sorted(set(inputs))}))
    print('Saved:',root/'paths_first.png',root/'paths_final.png',root/'cost_time.png',folder)


if __name__=='__main__':
    main()
