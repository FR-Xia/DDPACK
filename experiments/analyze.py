"""Generate publication tables from real, paired MATLAB/native solver outputs."""
from pathlib import Path
import argparse, csv, hashlib, json, shutil, platform
from datetime import date
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
NAMES = ['vibration','planar','terminal_defect','obstacle_2d','obstacle_3d',
         'reactor','pendulum','thermal','storage']
LABELS = ['Vibration','Planar','Terminal defect','Obstacle 2D','Obstacle 3D',
          'Reactor','Pendulum','Thermal','Storage']

def table(path, rows):
    with path.open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def main():
    a=argparse.ArgumentParser();a.add_argument('--matlab',type=Path,required=True)
    a.add_argument('--cpp',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
    a.add_argument('--measurement-date',type=date.fromisoformat,help='Actual measurement date (YYYY-MM-DD); distinct from analysis time')
    args=a.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    generated=ROOT/'DDPACK_Manual/generated';generated.mkdir(exist_ok=True)
    raw={};manifest=[]
    for lang,folder in [('matlab',args.matlab),('cpp',args.cpp)]:
        with (folder/'timings.csv').open() as f:raw[lang]=list(csv.DictReader(f))
        assert len(raw[lang])==45 and all(r['success'] in ('1','true') for r in raw[lang])
        shutil.copyfile(folder/'timings.csv',out/(lang+'_timings.csv'))
        environment=folder/'environment.txt' if lang=='matlab' else folder/'vibration/environment.txt'
        shutil.copyfile(environment,out/(lang+'_environment.txt'))
        snapshot=out/'raw'/lang;snapshot.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(folder/'timings.csv',snapshot/'timings.csv')
        if lang=='matlab':shutil.copyfile(environment,snapshot/'environment.txt')
        for name in NAMES:
            target=snapshot/name;target.mkdir(exist_ok=True)
            for filename in ['states.csv','controls.csv','time.csv','metrics.json','config.json','parameters.json','environment.txt','line_search_trials.csv','merit.csv','refinement_starts.csv','refinement_penalty_history.csv']:
                source=folder/name/filename
                if source.exists():shutil.copyfile(source,target/filename)
        for file in sorted(folder.rglob('*')):
            if file.is_file():manifest.append(dict(language=lang,file=file.relative_to(folder).as_posix(),sha256=hashlib.sha256(file.read_bytes()).hexdigest()))
    comparisons=[];physical=[];configs=[];times={};trajectories={}
    for name,label in zip(NAMES,LABELS):
        arrays={};metrics={}
        for lang,folder in [('matlab',args.matlab),('cpp',args.cpp)]:
            arrays[lang]={key:np.loadtxt(folder/name/(key+'.csv'),delimiter=',',ndmin=2) for key in ['states','controls','time']}
            metrics[lang]=json.loads((folder/name/'metrics.json').read_text())
            values=[float(r['seconds']) for r in raw[lang] if r['case']==name]
            assert len(values)==5 and np.isfinite(values).all() and min(values)>0
            times[lang,name]=np.quantile(values,[.25,.5,.75])
        x=arrays['matlab']['states'];xc=arrays['cpp']['states']
        u=arrays['matlab']['controls'][:,:-1];uc=arrays['cpp']['controls'][:,:-1]
        assert x.shape==xc.shape and u.shape==uc.shape
        dx=float(np.max(np.abs(x-xc)));du=float(np.max(np.abs(u-uc)))
        rx=dx/max(1,float(np.max(np.abs(x))));ru=du/max(1,float(np.max(np.abs(u))))
        assert rx<=2e-6
        assert ru<=2e-6
        m=metrics['matlab'];c=metrics['cpp'];assert m['success'] and c['success']
        if name in ['obstacle_2d','obstacle_3d']:
            for metric in [m,c]:
                assert metric['analytic_zoh_clearance_lower_bound']>0
                assert metric['analytic_zoh_terminal_error']<.005
            for field in ['analytic_zoh_clearance_lower_bound','analytic_zoh_terminal_error']:
                assert abs(m[field]-c[field])<1e-9,(name,field)
        parameters=json.loads((args.matlab/name/'parameters.json').read_text())
        def objective(states,controls,time):
            target=np.asarray(parameters['target']).reshape(-1,1);d=states-target
            q=np.asarray(parameters['Q_s']).reshape(d.shape[0],d.shape[0])
            f=np.asarray(parameters['Q_f']).reshape(d.shape[0],d.shape[0])
            r=np.asarray(parameters['R']).reshape(controls.shape[0],controls.shape[0])
            density=.5*(np.sum(d[:,:-1]*(q@d[:,:-1]),axis=0)+np.sum(controls*(r@controls),axis=0))
            if parameters['algorithm']=='penalty_free_time_ddp':density=density+1
            return float(.5*d[:,-1]@f@d[:,-1]+np.sum(np.diff(time.ravel())*density))
        jm=objective(x,u,arrays['matlab']['time']);jc=objective(xc,uc,arrays['cpp']['time'])
        assert abs(jc-c['returned_cost'])<1e-9*max(1,abs(jc))
        row=dict(case=name,max_state_error=dx,scaled_state_error=rx,max_active_control_error=du,scaled_control_error=ru,
                 absolute_objective_difference=abs(jm-jc),matlab_recomputed_objective=jm,cpp_recomputed_objective=jc,
                 matlab_reported_minus_recomputed_objective=m['returned_objective']-jm,
                 absolute_final_time_difference=abs(m['final_time']-c['final_time']))
        for lang in ['matlab','cpp']:
            q=times[lang,name]
            row.update({lang+'_q25_seconds':q[0],lang+'_median_seconds':q[1],lang+'_q75_seconds':q[2]})
        row['median_ratio_matlab_over_cpp']=times['matlab',name][1]/times['cpp',name][1]
        comparisons.append(row)
        for lang,metric in metrics.items():
            native=lang=='cpp'
            physical.append(dict(case=name,language=lang,success=metric['success'],terminal_error=metric['terminal_error'],
                returned_cost=jc if native else jm,reported_objective=metric['returned_cost' if native else 'returned_objective'],final_time=metric['final_time'],
                euler_defect=metric['euler_defect' if native else 'absolute_dynamics_defect'],
                rk4_defect=metric['rk4_defect' if native else 'high_accuracy_absolute_dynamics_defect'],
                minimum_segment_clearance=metric['minimum_clearance' if native else 'minimum_segment_obstacle_clearance'],
                control_violation=metric['control_violation' if native else 'maximum_control_constraint_violation'],
                analytic_zoh_clearance_lower_bound=metric.get('analytic_zoh_clearance_lower_bound'),
                analytic_zoh_terminal_error=metric.get('analytic_zoh_terminal_error')))
        conf=json.loads((args.cpp/name/'config.json').read_text())
        for key in ['algorithm','tf','dt','max_iterations','alpha','epsilon','rho','update_epsilon']:
            assert conf[key]==parameters[key], (name,key,conf[key],parameters[key])
        for key in ['geometric_refinement_iterations','geometric_refinement_knots','geometric_control_rate_weight','geometric_refinement_free_time','minimum_time','maximum_time']:
            if key in parameters and parameters[key] is not None: assert conf[key]==parameters[key],(name,key)
        shutil.copyfile(args.matlab/name/'parameters.json',out/(name+'_parameters.json'))
        conf.update(state_dimension=x.shape[0],control_dimension=u.shape[0],state_samples=x.shape[1]);configs.append(conf)
        trajectories[name]=arrays
    table(out/'comparison.csv',comparisons);table(out/'physical_metrics.csv',physical)
    table(out/'configuration.csv',configs);table(out/'input_sha256.csv',manifest)
    sources=[]
    for folder in ['matlab/src','matlab/examples','matlab/benchmarks','matlab/tools','cpp','experiments']:
        for file in sorted((ROOT/folder).rglob('*')):
            if file.is_file() and (file.suffix in {'.m','.cpp','.hpp','.py','.cmake','.ps1'} or file.name=='CMakeLists.txt') and 'results' not in file.parts:
                sources.append(dict(file=file.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(file.read_bytes()).hexdigest()))
    sources.append(dict(file='CMakeLists.txt',sha256=hashlib.sha256((ROOT/'CMakeLists.txt').read_bytes()).hexdigest()))
    table(out/'source_sha256.csv',sources)
    (out/'measurement_context.txt').write_text(f'Measured {args.measurement_date or "not recorded"}, Asia/Shanghai. Windows 11; Intel Core i5-12500H; MSVC 19.44 Release x64; Eigen 5.0.0; C++17.\n'
        'MATLAB threading is recorded in matlab_environment.txt. Sequential MATLAB then C++; no concurrent solver or compilation jobs.\n'
        '3 warmups, 5 repeats, quantiles via NumPy linear interpolation; descriptive timing only.\n'
        f'Analysis: Python {platform.python_version()}, NumPy {np.__version__}, Matplotlib {matplotlib.__version__}.\n',encoding='utf-8')
    plt.rcParams.update({'font.size':9,'pdf.fonttype':42,'ps.fonttype':42})
    fig,ax=plt.subplots(figsize=(8,3.6),layout='constrained');y=np.arange(9)
    for lang,color,offset in [('matlab','#165D8D',-.14),('cpp','#C45A18',.14)]:
        q=np.array([times[lang,n] for n in NAMES])*1000
        ax.errorbar(q[:,1],y+offset,xerr=np.array([q[:,1]-q[:,0],q[:,2]-q[:,1]]),fmt='o',capsize=3,label=lang.upper(),color=color)
    ax.set_yticks(y,LABELS);ax.invert_yaxis();ax.set_xscale('log');ax.set_xlabel('Public solve time (ms): median and interquartile range, n = 5')
    ax.grid(axis='x',alpha=.25);ax.legend();fig.savefig(out/'timing.pdf');fig.savefig(out/'timing.png',dpi=180);plt.close(fig)
    fig,(ax,bx)=plt.subplots(1,2,figsize=(8,3.2),layout='constrained')
    for lang,style in [('matlab','-'),('cpp','--')]:
        x=trajectories['obstacle_2d'][lang]['states'];ax.plot(x[0],x[1],style,label=lang.upper())
    ax.add_patch(plt.Circle((2.5,0),.25,color='gray',alpha=.3,label='Obstacle + margin'))
    ax.set(xlabel='x (m)',ylabel='y (m)',title='2D obstacle: paired trajectories');ax.axis('equal');ax.legend(fontsize=7)
    x=trajectories['obstacle_3d'];delta=np.max(np.abs(x['matlab']['states']-x['cpp']['states']),axis=0)
    bx.plot(x['matlab']['time'].ravel(),delta);bx.set(xlabel='Physical time (s)',ylabel='Max absolute state-component difference',title='3D obstacle: final trajectory agreement');bx.grid(alpha=.25)
    fig.savefig(out/'trajectory_agreement.pdf');fig.savefig(out/'trajectory_agreement.png',dpi=180);plt.close(fig)
    for name in ['timing.pdf','trajectory_agreement.pdf']:shutil.copyfile(out/name,generated/name)
    lines=[r'\begin{table}[htbp]\centering\small',r'\caption{同输入完整求解对比：耗时中位数及四分位区间，单位 ms；每例每语言 5 次。}',r'\begin{tabular}{lrrr}\toprule',r'案例 & MATLAB & C++ & 中位数比值\\\midrule']
    for name,label,row in zip(NAMES,LABELS,comparisons):
        m=times['matlab',name]*1000;c=times['cpp',name]*1000
        lines.append(f'{label} & {m[1]:.2f} [{m[0]:.2f}, {m[2]:.2f}] & {c[1]:.2f} [{c[0]:.2f}, {c[2]:.2f}] & {row["median_ratio_matlab_over_cpp"]:.2f}'+r'\\')
    lines += [r'\bottomrule\end{tabular}\end{table}',r'\begin{table}[htbp]\centering\small',r'\caption{双语言的轨迹误差与物理终端误差。$\Delta x_\infty$、$\Delta u_\infty$ 为绝对分量最大差。}',r'\begin{tabular}{lrrrr}\toprule',r'案例 & $\Delta x_\infty$ & $\Delta u_\infty$ & 终端误差 M & 终端误差 C++\\\midrule']
    for i,(label,row) in enumerate(zip(LABELS,comparisons)):
        lines.append(f'{label} & {row["max_state_error"]:.2e} & {row["max_active_control_error"]:.2e} & {physical[2*i]["terminal_error"]:.2e} & {physical[2*i+1]["terminal_error"]:.2e}'+r'\\')
    lines += [r'\bottomrule\end{tabular}\end{table}',r'\begin{table}[htbp]\centering\small',r'\caption{独立四子步 RK4 缺陷、返回目标差与几何净空。无障碍案例以 -- 标记不适用。}',r'\begin{tabular}{lrrrrr}\toprule',r'案例 & RK4 M & RK4 C++ & $|\Delta J|$ & 净空 M & 净空 C++\\\midrule']
    for i,(label,row) in enumerate(zip(LABELS,comparisons)):
        m=physical[2*i];c=physical[2*i+1]
        def fmt(v):return '--' if v is None else f'{v:.2e}'
        lines.append(f'{label} & {fmt(m["rk4_defect"])} & {fmt(c["rk4_defect"])} & {fmt(row["absolute_objective_difference"])} & {fmt(m["minimum_segment_clearance"])} & {fmt(c["minimum_segment_clearance"])}'+r'\\')
    lines += [r'\bottomrule\end{tabular}\end{table}']
    (generated/'language_comparison.tex').write_text('\n'.join(lines).replace('[htbp]','[H]')+'\n',encoding='utf-8')
    compact=[r'\begin{table}[htbp]\centering\small',r'\caption{Paired public-solve medians (ms), five measured repeats after three warmups, and maximum state differences.}',r'\begin{tabular}{lrrr}\toprule',r'Case & MATLAB & C++ & $\|\Delta x\|_\infty$\\\midrule']
    for name,label,row in zip(NAMES,LABELS,comparisons):compact.append(f'{label} & {1000*times["matlab",name][1]:.2f} & {1000*times["cpp",name][1]:.2f} & {row["max_state_error"]:.2e}'+r'\\')
    compact += [r'\bottomrule\end{tabular}\end{table}']
    manuscript_generated=ROOT/'manuscript/generated'
    if manuscript_generated.is_dir():
        (manuscript_generated/'language_comparison.tex').write_text('\n'.join(compact)+'\n',encoding='utf-8')
    print('Generated nine paired comparisons, 18 physical records, 90 raw timings and two figures.')

if __name__=='__main__':main()
