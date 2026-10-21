#!/usr/bin/env python3
"""Independent synthetic plant + actual ROS driver logic (stub transport).

All simulated NPCs are visible in this benchmark. No MORAI success is implied.
Randomizes speed/count/size, latency, ID turnover and bounded detection gaps.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import random
import sys
import time
import types
import yaml
import numpy as np
from purepursuit_mgeo.path import PathPoint,load_mgeo_path
from roundabout_mission.core import Decision,Ego,Track


ROOT=Path(__file__).resolve().parents[4]
PACKAGE=Path(__file__).resolve().parents[1]


def fixture():
    path=load_mgeo_path(str(ROOT/'data/routes/2026_molit_comp_global_path.txt'))
    links={l['idx']:l for l in json.loads((ROOT/'data/mgeo/R_KR_PR_K-city_2025/link_set.json').read_text())}
    circ=[]
    for key in ('A2256W000134','A2256W000135','A2256W000132','A2256W000133'):
        for p in links[key]['points']:
            if not circ or math.hypot(p[0]-circ[-1].x,p[1]-circ[-1].y)>1e-6:
                circ.append(PathPoint(*p))
    c=yaml.safe_load((PACKAGE/'config/roundabout.yaml').read_text())
    c.update(path_file=str(ROOT/'data/routes/2026_molit_comp_global_path.txt'),
        mission_indices=dict(approach=1720,stop_line=1788,entry=1800,conflict_start=1808,conflict_end=1860,merge_complete=1865,exit=1900),
        circulating_conflict_indices=[30,82],roi_map=[-126,-78,319,368],
        enable_control=True,log_dir='',command_topic='/ctrl_cmd')
    c.update(approach_speed=2.,merge_speed=3.,roundabout_speed=3.,exit_speed=2.)
    return path,circ,c


def driver_module(c,clock):
    class Publisher:
        def __init__(self,*a,**kw):self.last=None
        def publish(self,msg):self.last=msg
    class Message:
        def __init__(self,**kw):self.__dict__.update(kw)
    ros=types.ModuleType('rospy')
    ros.get_param=lambda *a:c
    ros.Publisher=Publisher
    ros.Subscriber=ros.Timer=ros.on_shutdown=lambda *a,**kw:None
    ros.Duration=lambda x:x
    ros.loginfo=ros.loginfo_throttle=ros.logwarn_throttle=lambda *a:None
    ros.Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_sec=lambda:clock[0]))
    sys.modules['rospy']=ros
    for package,name in [('nav_msgs','Odometry'),('std_msgs','String'),('morai_msgs','CtrlCmd')]:
        sys.modules[package]=types.ModuleType(package)
        msg=types.ModuleType(package+'.msg');setattr(msg,name,Message);sys.modules[package+'.msg']=msg
    spec=importlib.util.spec_from_file_location('bench_driver',PACKAGE/'scripts/roundabout_test_driver.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.time=types.SimpleNamespace(monotonic=lambda:clock[0])
    return module


def polygon(x,y,yaw,length,width,offset=0):
    ca,sa=math.cos(yaw),math.sin(yaw)
    return [(x+(a+offset)*ca-b*sa,y+(a+offset)*sa+b*ca)
            for a,b in [(length/2,width/2),(-length/2,width/2),(-length/2,-width/2),(length/2,-width/2)]]


def overlap(a,b):
    # Independent polygon-edge projection, using true states and ZERO margin.
    for poly in (a,b):
        for p,q in zip(poly,poly[1:]+poly[:1]):
            nx,ny=q[1]-p[1],p[0]-q[0]
            aa=[x*nx+y*ny for x,y in a];bb=[x*nx+y*ny for x,y in b]
            if max(aa)<min(bb) or max(bb)<min(aa):return False
    return True


def run(seed,seconds=100.,legacy=False,count=None,perturb=True,start_index=1778):
    rng=random.Random(seed);path,circ,c=fixture()
    c['trajectory_prediction_enabled']=not legacy
    if legacy:
        c.update(merge_speed=2.,roundabout_speed=2.,test_speed_limit=2.,maximum_merge_speed=2.)
    d=Decision(path,circ,c)
    # Benchmark ground truth uses piecewise-linear interpolation directly from
    # the reference points, independent of MergePredictor's future hypotheses.
    xy=np.array([(p.x,p.y) for p in circ]);ds=np.linalg.norm(np.diff(xy,axis=0),axis=1)
    arclength=np.r_[0.,np.cumsum(ds)];length=arclength[-1]
    def npc(s):
        s%=length;i=min(len(ds)-1,int(np.searchsorted(arclength,s,side='right')-1))
        t=(s-arclength[i])/ds[i];p=xy[i]+t*(xy[i+1]-xy[i])
        yaw=math.atan2(xy[i+1,1]-xy[i,1],xy[i+1,0]-xy[i,0])
        return float(p[0]),float(p[1]),yaw
    n=rng.randint(1,4) if count is None else count
    # Different but nearly spaced cars; unequal speeds may cause NPC/NPC overlap.
    # Only ego collisions are scored; this is NOT an interactive traffic model.
    cars=[dict(s=(i*length/max(n,1)+rng.uniform(0,12))%length,v=rng.uniform(2.,7.),
               length=rng.uniform(3.,6.5),width=rng.uniform(1.6,2.3),id=i+1) for i in range(n)]
    idx=start_index;e=Ego(path[idx].x,path[idx].y,math.atan2(path[idx+2].y-path[idx].y,path[idx+2].x-path[idx].x),0.)
    clock=[1.];module=driver_module(c,clock);driver=module.Driver()
    dt=.05;delay=.1 if perturb else 0.;queue=[]
    throttle_gain=rng.uniform(3.5,5.) if perturb else 4.
    brake_gain=rng.uniform(5.5,7.) if perturb else 6.
    drag=rng.uniform(.06,.10) if perturb else .08
    states=[];reasons={};max_error=0.;latencies=[];entered=False;collision=False;r={}
    approach_stop_commands=0;moving_emergency_commands=0
    wall_start=time.perf_counter()
    for k in range(int(seconds/dt)):
        now=1+k*dt;clock[0]=now
        if k%2==0:
            observed=[]
            for car in cars:
                # Bounded two-frame dropout, not a falsely asserted clear space.
                if perturb and int(now*10)%97 in (4,5):continue
                x,y,yaw=npc(car['s']+car['v']*max(0,now-1-delay))
                err=rng.uniform(-.04,.04) if perturb else 0.
                v=car['v']+(rng.uniform(-.1,.1) if perturb else 0.)
                tid=car['id']+(int((now-1)/3)*100 if perturb else 0)
                observed.append(Track(tid,x+err,y-err,yaw,car['length'],car['width'],v*math.cos(yaw),v*math.sin(yaw)))
            d.update_tracks(observed,now-delay)
            start=time.perf_counter();r=d.step(e,now);latencies.append(time.perf_counter()-start)
            if not states or states[-1]!=r['state']:states.append(r['state'])
            reason=r['decision_reason'];reasons[reason]=reasons.get(reason,0)+1
            if r.get('decision_phase')=='APPROACH' and r['stop_required']:approach_stop_commands+=1
            if r['emergency'] and e.speed>.15:moving_emergency_commands+=1
            driver.status=r;driver.status_stamp=now;driver.status_wall=now
        driver.ego=Ego(e.x,e.y,e.yaw,e.speed);driver.odom_stamp=now;driver.odom_wall=now
        driver.tick(None);cmd=driver.preview.last
        queue.append((now,cmd))
        effective=queue[0][1]
        while len(queue)>1 and queue[1][0]<=now-delay:queue.pop(0);effective=queue[0][1]
        e.speed=max(0.,e.speed+(throttle_gain*effective.accel-brake_gain*effective.brake-drag*e.speed)*dt)
        e.x+=e.speed*math.cos(e.yaw)*dt;e.y+=e.speed*math.sin(e.yaw)*dt
        e.yaw+=e.speed*math.tan(effective.steering)/c['wheelbase_m']*dt
        epoly=polygon(e.x,e.y,e.yaw,c['ego_length'],c['ego_width'],c['ego_front_m']-c['ego_length']/2)
        for car in cars:
            x,y,yaw=npc(car['s']+car['v']*(now-1+dt))
            if overlap(epoly,polygon(x,y,yaw,car['length'],car['width'])):collision=True
        err,s,_,_=d.path.project(e.x,e.y);max_error=max(max_error,err)
        if s>=d.m['conflict_start']:entered=True
        if collision or r.get('state')=='EXIT':break
    return dict(seed=seed,legacy=legacy,vehicles=n,perturb=perturb,
        outcome='collision' if collision else ('exit' if r.get('state')=='EXIT' else ('timeout_after_entry' if entered and e.speed>=.15 else ('stopped_inside' if entered else 'waiting'))),
        simulated_seconds=round(k*dt,2),max_path_error=round(max_error,3),states=states,
        reasons=reasons,final_reason=r.get('decision_reason'),final_speed=e.speed,final_s=s,
        gate_ms_p95=round(float(np.percentile(latencies,95))*1000,2),
        wall_seconds=round(time.perf_counter()-wall_start,2),
        approach_stop_commands=approach_stop_commands,moving_emergency_commands=moving_emergency_commands,
        start_index=start_index,merge_speed=c['merge_speed'],traffic=cars,
        model=dict(throttle_gain=throttle_gain,brake_gain=brake_gain,drag=drag,latency=delay))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--seed',type=int,default=1)
    p.add_argument('--seconds',type=float,default=100);p.add_argument('--legacy',action='store_true')
    p.add_argument('--count',type=int);p.add_argument('--ideal',action='store_true')
    p.add_argument('--start-index',type=int,default=1778)
    a=p.parse_args();print(json.dumps(run(a.seed,a.seconds,a.legacy,a.count,not a.ideal,a.start_index)))
