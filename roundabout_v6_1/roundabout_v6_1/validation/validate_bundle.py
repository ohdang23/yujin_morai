"""Meaningful decision, relative-following, rear-regression and install checks."""
import argparse
import ast
import copy
import csv
import hashlib
import importlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import threading
import types
from collections import deque
from types import SimpleNamespace as P
from unittest.mock import patch
import yaml

def load(bundle):
    source=bundle/'roundabout_mission/src/roundabout_mission'
    alias='checked_roundabout'
    spec=importlib.util.spec_from_file_location(alias,source/'__init__.py',submodule_search_locations=[str(source)])
    pkg=importlib.util.module_from_spec(spec);sys.modules[alias]=pkg;spec.loader.exec_module(pkg)
    new=importlib.import_module(alias+'.core')
    rspec=importlib.util.spec_from_file_location(alias+'.reference_core',bundle/'validation/reference_v5_core.py')
    old=importlib.util.module_from_spec(rspec);sys.modules[rspec.name]=old;rspec.loader.exec_module(old)
    return pkg,new,old

def validate(bundle):
    pkg,new,old=load(bundle)
    version=pkg.VERSION
    config=bundle/('v61_speedtest.yaml' if version=='6.1.0' else 'v6_speedtest.yaml')
    original=yaml.safe_load((bundle/'original_v5_speedtest.yaml').read_text())
    cfg=yaml.safe_load(config.read_text())
    assert cfg['minimum_gap']==1.0 and cfg['time_headway']==1.5
    assert cfg['emergency_gap_m']==original['emergency_gap_m']==1.5
    assert cfg['static_minimum_gap_m']==original['minimum_gap']==5.
    allowed={'minimum_gap','static_minimum_gap_m','version'}
    if version=='6.1.0':allowed.update({'lead_relative_following_enabled','lead_follow_alignment_cos_min','lead_follow_lateral_tolerance_m','lead_relative_reaction_s'})
    differences={k for k in set(cfg)|set(original) if cfg.get(k)!=original.get(k)}
    assert differences<=allowed,differences
    if version=='6.0.0':
        expected=(bundle/'validation/reference_v5_core.py').read_text().replace("gap-c['minimum_gap']))))", "gap-c.get('static_minimum_gap_m', c['minimum_gap'])))))",1)
        assert (bundle/'roundabout_mission/src/roundabout_mission/core.py').read_text()==expected

    py=list((bundle/'roundabout_mission').rglob('*.py'))
    for file in py:ast.parse(file.read_text(),filename=str(file),feature_version=(3,8))
    for file in (bundle/'install.sh',bundle/'run_trial.sh'):
        subprocess.run(['bash','-n',str(file)],check=True)

    c=copy.deepcopy(cfg)
    c.update(mission_indices=dict(approach=5,stop_line=20,entry=25,conflict_start=46,conflict_end=56,merge_complete=75,exit=100),
             circulating_conflict_indices=[145,155],roi_map=[-10,90,-160,160],
             entry_decision_stop_index=20,trajectory_prediction_enabled=False,merge_speed=3.4)
    path=[P(x=.5*i,y=0.) for i in range(2001)]
    circ=[P(x=26.,y=float(y)) for y in range(-150,151)]
    extra={'decision_reason_detail','lead_relative_following_active','lead_prediction_stop_suppressed',
           'lead_close_gap_stop_suppressed','lead_prediction_suppressed_ids','lead_gap_control_mode',
           'lead_current_separation_m','lead_closing_speed_mps','lead_relative_speed_cap_mps'}

    def engine(module,state='COMMIT',settings=None):
        d=module.Decision(path,circ,copy.deepcopy(settings or c))
        d.state=state;d.entry_stop_completed=state in ('COMMIT','FOLLOW')
        d.target=3.4
        return d
    def step(module,d,ego,tracks=(),now=1.,fresh=True,obstacles=True):
        d.update_tracks([module.Track(*v) for v in tracks],now)
        row=d.step(module.Ego(*ego),now,fresh,obstacles)
        json.dumps(row,allow_nan=False)
        return row
    def lead(gap,speed=2.,x=14.,yaw=0.,vy=0.,id=10):
        # Track centre: ego reference + front offset + projected box radius + gap.
        radius=old.projected_radius(4.5,1.8,yaw,(1.,0.))
        return (id,x+c['ego_front_m']+radius+gap,0.,yaw,4.5,1.8,speed,vy)
    def identical(a,b):
        assert a=={k:v for k,v in b.items() if k not in extra},[(k,a.get(k),b.get(k)) for k in set(a)|set(b) if k not in extra and a.get(k)!=b.get(k)]

    checks=[];examples={}
    def check(label,fn):
        fn();checks.append(label)

    def base_gap():
        settings=copy.deepcopy(c);settings['minimum_gap']=5.
        a=step(old,engine(old,settings=settings),(14.,0.,0.,2.),[lead(4.)])
        b=step(new,engine(new),(14.,0.,0.,2.),[lead(4.)])
        assert abs(a['desired_gap']-8.)<1e-8 and abs(b['desired_gap']-4.)<1e-8
        assert b['target_speed']>a['target_speed']
        examples['base_gap']={k:b.get(k) for k in ('desired_gap','lead_gap','target_speed','stop_required','emergency')}
    check('5 m -> 1 m changes the running following request, not just YAML text',base_gap)
    def static_buffer():
        prior=copy.deepcopy(c);prior['minimum_gap']=5.
        a=step(old,engine(old,settings=prior),(14.,0.,0.,2.),[lead(4.,speed=0.)])
        b=step(new,engine(new),(14.,0.,0.,2.),[lead(4.,speed=0.)])
        assert a['target_speed']==b['target_speed']==0.
    check('the original 5 m stationary-obstacle buffer stays unchanged',static_buffer)

    # Compare full decision results on all inputs outside the new front-only rule.
    for label,ego,tracks,state,fresh,obstacles in (
        ('empty committed road',(14.,0.,0.,2.),[],'COMMIT',True,True),
        ('rear approaching after COMMIT',(14.,0.,0.,2.),[(20,10.,0.,0.,4.5,1.8,5.,0.)],'COMMIT',True,True),
        ('rear approaching before authorization',(10.,0.,0.,0.),[(20,6.,0.,0.,4.5,1.8,5.,0.)],'WAIT',True,True),
        ('stationary front obstacle',(14.,0.,0.,2.),[lead(1.3,speed=0.)],'COMMIT',True,True),
        ('present front box overlap',(14.,0.,0.,3.4),[lead(-.1)],'COMMIT',True,True),
        ('side crossing vehicle',(14.,0.,0.,2.),[(30,16.,2.,-math.pi/2,4.5,1.8,0.,-3.)],'COMMIT',True,True),
        ('oblique moving front is not a following exception',(14.,0.,0.,3.4),[lead(1.3,speed=1.,yaw=math.pi/4,vy=1.)],'COMMIT',True,True),
        ('front risk before authorization',(10.,0.,0.,3.4),[lead(1.3,x=10.)],'WAIT',True,True),
        ('stale odometry',(14.,0.,0.,2.),[lead(4.)],'COMMIT',False,True),
        ('stale perception',(14.,0.,0.,2.),[lead(4.)],'COMMIT',True,False),
        ('off path',(14.,4.,0.,2.),[],'COMMIT',True,True),
    ):
        a=step(old,engine(old,state),ego,tracks,fresh=fresh,obstacles=obstacles)
        b=step(new,engine(new,state),ego,tracks,fresh=fresh,obstacles=obstacles)
        identical(a,b);checks.append(label+' preserves V6 baseline decision')
        if 'rear approaching' in label:
            examples[label]={k:b.get(k) for k in ('state','stop_required','emergency','minimum_ttc')}

    if version=='6.1.0':
        def closing():
            ego=(14.,0.,0.,3.4);tr=[lead(1.3)]
            a=step(old,engine(old),ego,tr);b=step(new,engine(new),ego,tr)
            assert a['emergency'] and a['stop_required'] and a['target_speed']==0
            assert b['lead_prediction_stop_suppressed'] and b['lead_relative_following_active']
            assert not b['emergency'] and not b['stop_required'] and 0<b['target_speed']<=2.
            assert b['lead_closing_speed_mps']>0.
            examples['closing_front']={k:b.get(k) for k in ('minimum_ttc','target_speed','stop_required','emergency','lead_prediction_stop_suppressed','lead_relative_speed_cap_mps')}
        check('imminent predicted moving lead uses a nonzero speed limit, not a full stop',closing)
        def pulling_away():
            b=step(new,engine(new),(14.,0.,0.,1.),[lead(1.2)])
            assert not b['stop_required'] and not b['emergency']
            assert b['target_speed']==2. and b['lead_closing_speed_mps']==0.
        check('near moving front pulling away does not stop the ego',pulling_away)
        def matching():
            b=step(new,engine(new),(14.,0.,0.,2.),[lead(1.2)])
            assert not b['stop_required'] and not b['emergency'] and b['target_speed']==2.
        check('matching a close moving front does not stop the ego',matching)
        def permutation():
            prior=None
            for i in (10,999,73):
                b=step(new,engine(new),(14.,0.,0.,3.4),[lead(1.3,id=i)])
                values=[b[k] for k in ('target_speed','emergency','stop_required','lead_relative_following_active')]
                if prior is not None:assert values==prior
                prior=values
        check('track ID changes do not change front following behavior',permutation)
        def disabled():
            cc=copy.deepcopy(c);cc['lead_relative_following_enabled']=False
            a=step(old,engine(old,settings=cc),(14.,0.,0.,3.4),[lead(1.3)])
            b=step(new,engine(new,settings=cc),(14.,0.,0.,3.4),[lead(1.3)])
            identical(a,b)
        check('disabling the front-only feature restores V6',disabled)
        def speed_cap():
            b=step(new,engine(new),(14.,0.,0.,3.4),[lead(8.)])
            dv=b['lead_relative_speed_cap_mps']-2.
            consumed=dv*c['lead_relative_reaction_s']+dv*dv/(2*c['comfortable_decel'])
            assert consumed <= 8.-c['minimum_gap']+1e-8
        check('relative closing cap includes response delay and stopping distance',speed_cap)

        def module_file(name,file):
            spec=importlib.util.spec_from_file_location(name,file)
            m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m)
            return m
        pp=module_file('validation_pursuit_path',bundle/'validation/reference_path.py')
        pid=module_file('validation_pedal_pid',bundle/'validation/reference_longitudinal_controller.py')
        rospy=types.ModuleType('rospy')
        clock={'now':1.}
        rospy.Time=P(now=lambda:P(to_sec=lambda:clock['now']))
        rospy.loginfo_throttle=rospy.logwarn_throttle=lambda *a:None
        modules={'rospy':rospy,'roundabout_mission':pkg,'roundabout_mission.core':new}
        for name in ('nav_msgs','nav_msgs.msg','std_msgs','std_msgs.msg','morai_msgs','morai_msgs.msg','purepursuit_mgeo','purepursuit_mgeo.path','path_planning','path_planning.longitudinal_controller'):
            modules[name]=types.ModuleType(name)
        modules['nav_msgs.msg'].Odometry=object
        modules['std_msgs.msg'].String=lambda **kw:P(**kw)
        modules['morai_msgs.msg'].CtrlCmd=type('CtrlCmd',(),{})
        modules['purepursuit_mgeo.path'].MgeoPurePursuit=pp.MgeoPurePursuit
        modules['purepursuit_mgeo.path'].load_mgeo_path=pp.load_mgeo_path
        modules['path_planning.longitudinal_controller'].PedalSpeedController=pid.PedalSpeedController
        modules['roundabout_mission.ros_io']=importlib.import_module('checked_roundabout.ros_io')
        modules['roundabout_mission.integration']=importlib.import_module('checked_roundabout.integration')
        with patch.dict(sys.modules,modules):
            driver=module_file('validation_actual_driver',bundle/'roundabout_mission/scripts/roundabout_test_driver.py')
            preparer=module_file('validation_actual_preparer',bundle/'roundabout_mission/scripts/prepare_roundabout_v6.py')
        def prepare_and_log():
            with tempfile.TemporaryDirectory() as tmp:
                area=Path(tmp);path_file=area/'path.txt';circ_file=area/'circ.txt'
                path_file.write_text(''.join('%s %s 0\n'%(p.x,p.y) for p in path))
                circ_file.write_text(''.join('%s %s 0\n'%(p.x,p.y) for p in circ))
                previous=dict(c,path_file=str(path_file),circulating_path_file=str(circ_file),minimum_gap=5.,version='5.0.0')
                input_file=area/'original.yaml';output_file=area/'new.yaml'
                input_file.write_text(yaml.safe_dump(previous));before=input_file.read_bytes()
                preparer.prepare(str(input_file),str(output_file))
                prepared=yaml.safe_load(output_file.read_text())
                assert prepared['minimum_gap']==1. and prepared['static_minimum_gap_m']==5.
                assert prepared['version']=='6.1.0' and input_file.read_bytes()==before
                for key in ('speed_kp','speed_ki','speed_kd','merge_speed','roundabout_speed','exit_speed','mission_indices'):
                    assert prepared[key]==previous[key]
                row=step(new,engine(new),(14.,0.,0.,3.4),[lead(1.3)])
                log=modules['roundabout_mission.ros_io'].CsvLog(str(area),'decision');log.write(row);log.close()
                saved=list(csv.DictReader(next(area.glob('decision_*.csv')).open()))[0]
                assert saved['lead_prediction_stop_suppressed']=='True'
                assert saved['lead_gap_control_mode']=='relative_speed'
                assert saved['decision_reason_detail']=='lead_relative_following'
        check('custom YAML preparation preserves controls and CSV retains front-follow diagnostics',prepare_and_log)

        class Pub:
            def publish(self,msg):self.last=msg
        class Log:
            def write(self,row):self.last=row
        def control(settings):
            d=driver.Driver.__new__(driver.Driver)
            d.c=dict(settings,enable_control=True)
            d.lock=threading.RLock()
            d.pp=pp.MgeoPurePursuit([pp.PathPoint(p.x,p.y,0.) for p in path],3.,4.,.35,1.5)
            d.route=new.Polyline(d.pp.points)
            d.pid=pid.PedalSpeedController(kp=c['speed_kp'],ki=c['speed_ki'],kd=c['speed_kd'],nominal_dt=.05)
            d.accel=d.brake=0.
            d.pub=Pub();d.preview=Pub();d.diagnostics=Pub();d.log=Log()
            return d
        def tick(d,row,ego,now):
            clock['now']=now
            d.status=row;d.ego=new.Ego(*ego)
            d.odom_stamp=d.status_stamp=now
            d.odom_wall=d.status_wall=now
            d.last_wall=now-.05
            with patch.object(driver.time,'monotonic',return_value=now):d.tick(None)
            return d.pub.last
        def brake_without_stop():
            e=(14.,0.,0.,3.4)
            row=step(new,engine(new),e,[lead(1.3)])
            d=control(c);cmd=tick(d,row,e,1.)
            assert cmd.accel==0 and cmd.brake>0
            assert not d.log.last['emergency'] and d.log.last['actual_target_speed']==2.
            for state,x in (('WAIT',10.),('COMMIT',14.)):
                e=(x,0.,0.,2.)
                tr=[(20,x-4.,0.,0.,4.5,1.8,5.,0.)]
                for module in (old,new):
                    row=step(module,engine(module,state),e,tr)
                    d=control(c);cmd=tick(d,row,e,1.)
                    assert (cmd.brake==1.) == (state=='WAIT')
        check('actual PID driver brakes a closing front without Stop; rear hard-stop behavior stays unchanged',brake_without_stop)

        def closed_loop():
            cc=copy.deepcopy(c)
            cc['mission_indices']=dict(approach=5,stop_line=20,entry=25,conflict_start=1000,conflict_end=1020,merge_complete=1040,exit=1700)
            gate=engine(new,settings=cc);d=control(cc)
            x=14.;v=3.4;front_x=x+5.85+8.
            pending=deque([(0.,0.),(0.,0.)]);samples=[]
            for i in range(280):
                now=1.+i*.05
                front_v=2. if i<100 else (1.4 if i<180 else 2.5)
                tr=[(10+i//35,front_x,0.,0.,4.5,1.8,front_v,0.)]
                e=(x,0.,0.,v);row=step(new,gate,e,tr,now)
                cmd=tick(d,row,e,now)
                gap=front_x-x-5.85
                samples.append(dict(t=now-1.,gap=gap,speed=v,front_speed=front_v,target=row['target_speed'],brake=cmd.brake))
                assert gap>0. and not row['emergency'] and not row['stop_required']
                pending.append((cmd.accel,cmd.brake));accel,brake=pending.popleft()
                # Explicit synthetic vehicle; never labelled MORAI ground truth.
                next_v=max(0.,v+(3.5*accel-5.*brake-(.1 if v>0 else 0.))*.05)
                x+=(v+next_v)*.025;v=next_v;front_x+=front_v*.05
            assert min(s['gap'] for s in samples)>=1.
            examples['closed_loop']=dict(frames=len(samples),minimum_gap_m=min(s['gap'] for s in samples),
                speed_command_delay_s=.1,model='synthetic 3.5*accel - 5.0*brake - 0.1 m/s^2; actual captured PID/Pure Pursuit and unchanged driver')
            with open(bundle/'validation/closed_loop.csv','w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=list(samples[0]));w.writeheader();w.writerows(samples)
        check('280 delayed closed-loop frames follow an accelerating/decelerating front without a predicted stop',closed_loop)

    # Stateful rear-only cases: ID changes, temporary invisibility and watchdogs.
    a=engine(old);b=engine(new);compared=0
    for i in range(80):
        now=1.+i*.05;ego=(14.+i*.01,0.,0.,2.)
        tracks=[] if 20<=i<23 else [(50+i,ego[0]-6.+.015*i,0.,0.,4.5,1.8,3.,0.)]
        ra=step(old,a,ego,tracks,now,fresh=not 35<=i<38,obstacles=not 55<=i<58)
        rb=step(new,b,ego,tracks,now,fresh=not 35<=i<38,obstacles=not 55<=i<58)
        identical(ra,rb);compared+=1
    checks.append('80 stateful rear-only frames match the V6 baseline exactly')

    with tempfile.TemporaryDirectory() as tmp:
        p=Path(tmp);target=p/'control/roundabout_mission'
        shutil.copytree(bundle/'roundabout_mission',target,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
        (target/'src/roundabout_mission/leftover_v54.py').write_text('sentinel=1\n')
        before=(target/'src/roundabout_mission/core.py').read_bytes()
        completed=subprocess.run(['bash',str(bundle/'install.sh'),str(target)],env=dict(os.environ,ROUNDABOUT_BACKUP_ROOT=str(p/'backups')),capture_output=True,text=True,check=True)
        backups=list((p/'backups').rglob('*.tar.gz'));assert len(backups)==1
        with tarfile.open(backups[0]) as archive:
            assert archive.extractfile('roundabout_mission/src/roundabout_mission/core.py').read()==before
            assert archive.extractfile('roundabout_mission/src/roundabout_mission/leftover_v54.py').read()==b'sentinel=1\n'
        assert not (target/'src/roundabout_mission/leftover_v54.py').exists()
        assert all(os.access(f,os.X_OK) for f in (target/'scripts').glob('*.py'))
    checks.append('installer preserves previous package and removes mixed-version leftovers')

    report=dict(version=version,status='pass',checks=checks,checks_count=len(checks),
                rear_stateful_decisions_compared=compared,
                original_v5_config_differences=sorted(differences),
                python38_syntax_files=len(py),examples=examples,
                runtime_limitations='Synthetic decision and mock installation checks; ROS Noetic/MORAI were not run. No physical no-collision guarantee.')
    (bundle/'validation/checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--bundle',type=Path,default=Path(__file__).resolve().parents[1])
    validate(p.parse_args().bundle.resolve())
