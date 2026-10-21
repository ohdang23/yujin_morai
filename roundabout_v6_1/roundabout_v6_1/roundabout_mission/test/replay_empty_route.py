#!/usr/bin/env python3
"""Offline kinematic regression, not MORAI validation. No ROS/network commands."""
import argparse
import math
import yaml
from purepursuit_mgeo.path import MgeoPurePursuit,load_mgeo_path
from path_planning.longitudinal_controller import PedalSpeedController
from roundabout_mission.core import Decision,Ego
from roundabout_mission.integration import apply_mission


def run(config):
    c=yaml.safe_load(open(config))
    pts=load_mgeo_path(c['path_file'])
    gate=Decision(pts,load_mgeo_path(c['circulating_path_file']),c)
    pp=MgeoPurePursuit(pts,c['wheelbase_m'],c['lookahead_min_m'],c['lookahead_gain'],c['goal_tolerance_m'])
    pid=PedalSpeedController(kp=c['speed_kp'],ki=c['speed_ki'],kd=c['speed_kd'],max_accel=c['max_accel_pedal'])
    e=Ego(pts[0].x,pts[0].y,math.atan2(pts[2].y-pts[0].y,pts[2].x-pts[0].x),0.)
    states=[]; max_error=0.;dt=.1
    for n in range(16000):
        now=1+n*dt
        gate.update_tracks([],now)
        r=gate.step(e,now)
        if not states or states[-1]!=r['state']:states.append(r['state'])
        if r['emergency']:raise AssertionError(r)
        steer,goal,_,_,_=pp.compute(e.x,e.y,e.yaw,e.speed)
        base=min(c['cruise_speed'],c['test_speed_limit'], math.sqrt(c['max_lateral_accel']/max(abs(math.tan(steer)/c['wheelbase_m']),1e-9)))
        target,stop,_=apply_mission(base,r,True)
        a,b=pid.compute(target,e.speed,now) if not stop else (0,1)
        # Simple synthetic plant. Coefficients are NOT a MORAI vehicle model.
        e.speed=max(0,e.speed+(4*a-6*b-.08*e.speed)*dt)
        e.x+=e.speed*math.cos(e.yaw)*dt;e.y+=e.speed*math.sin(e.yaw)*dt
        e.yaw+=e.speed*math.tan(steer)/c['wheelbase_m']*dt
        if n%20==0:
            err=gate.path.project(e.x,e.y)[0];max_error=max(max_error,err)
        if r['state']=='EXIT':
            assert 'COMMIT' in states and 'FOLLOW' in states,states
            print('PASS start->roundabout->exit; steps=%d simulated_seconds=%.1f max_path_error=%.3fm states=%s'%(n,n*dt,max_error,states))
            return
    raise AssertionError('Did not reach mission exit: '+str(r))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('config');run(p.parse_args().config)
