#!/usr/bin/env python3
"""Upgrade an existing mission YAML; leave original routes/config untouched."""
import argparse
import math
from pathlib import Path
import yaml
from purepursuit_mgeo.path import load_mgeo_path
from roundabout_mission import VERSION
from roundabout_mission.core import Decision, Ego, Track, swept_boxes


def tangent(points, i):
    a, b = points[max(0,i-2)], points[min(len(points)-1,i+2)]
    return math.atan2(b.y-a.y,b.x-a.x)


def prepare(existing, output, defaults):
    with open(defaults) as f:
        cfg = yaml.safe_load(f)
    with open(existing) as f:
        old = yaml.safe_load(f)
    cfg.update(old)
    # Local MORAI roundabout tuning validated during the v4.2 test session.
    # These values intentionally override the older v4/v4.1 runtime defaults.
    cfg.update(
        cruise_speed=2., max_lateral_accel=1.5, command_topic='/ctrl_cmd',
        track_hold_s=0.4, safe_confirm_s=0.3,
        front_time_buffer_s=0.8, rear_time_buffer_s=1.2,
        lost_recheck_s=0.3, lost_recheck_frames=2,
        reid_max_age_s=2.5, reid_distance_m=3.0, reid_velocity_delta_mps=3.0,
        merge_speed=5.0, roundabout_speed=5.0,
        minimum_merge_speed=5.0, maximum_merge_speed=5.0,
        test_speed_limit=5.0, max_accel=2.5,
        speed_kp=0.30, max_accel_pedal=1.0, max_accel_rate=3.0,
    )
    # The slow approach cruise and bounded candidate merge speeds are separate.
    merge_floor = max(0.1, float(cfg.get('minimum_merge_speed', 3.0)))
    merge_ceiling = max(merge_floor, float(cfg.get('maximum_merge_speed',4.0)))
    cfg['test_speed_limit'] = max(float(cfg.get('test_speed_limit', merge_floor)), merge_ceiling)
    cfg['merge_speed'] = max(float(cfg.get('merge_speed', merge_floor)), merge_floor)
    cfg['roundabout_speed'] = max(float(cfg.get('roundabout_speed', merge_floor)), merge_floor)
    # ETA must not assume a speed higher than the test driver can command.
    for key in ('approach_speed','merge_speed','roundabout_speed','exit_speed'):
        cfg[key] = min(cfg[key], cfg['test_speed_limit'])
    path, circ = [load_mgeo_path(cfg[k]) for k in ('path_file','circulating_path_file')]
    # Geometric overlap of two vehicle footprints, not unrelated hand-picked
    # indices. Use only the supplied approach..exit part of the original route.
    approach, exit_i = cfg['mission_indices']['approach'], cfg['mission_indices']['exit']
    if not 0 <= approach < exit_i < len(path):
        raise ValueError('Existing approach/exit indices must be configured')
    pairs=[]
    for i in range(approach,exit_i+1):
        e=Ego(path[i].x,path[i].y,tangent(path,i),0.)
        for j,t in enumerate(circ):
            # Circulating points represent vehicle centre for this estimate.
            track=Track(1,t.x,t.y,tangent(circ,j),cfg['ego_length'],cfg['ego_width'],0.,0.)
            if swept_boxes(e,track,cfg)[0] <= 0:
                pairs.append((i,j))
    if not pairs:
        raise ValueError('No footprint overlap between ego and circulating paths')
    lo, hi = min(i for i,j in pairs), max(i for i,j in pairs)
    cl, ch = min(j for i,j in pairs), max(j for i,j in pairs)
    if cl <= 0 or ch >= len(circ)-1:
        raise ValueError('Conflict crosses circulating-path seam; reorder that reference path')
    m=dict(cfg['mission_indices'])
    m['conflict_start'],m['conflict_end']=lo,hi
    m['merge_complete']=max(m['merge_complete'],hi+1)
    if not m['entry'] < lo < hi < m['merge_complete'] < exit_i:
        raise ValueError('Derived conflict does not fit mission order: '+str(m))
    cfg['mission_indices']=m
    cfg['circulating_conflict_indices']=[cl,ch]
    cfg['geometry_note']='v4 footprint overlap estimate; verify in MORAI; original ego CSV unchanged'
    cfg['version']=VERSION
    Decision(path,circ,cfg)
    with open(output,'x') as f:
        yaml.safe_dump(cfg,f,sort_keys=False)
    print('Created:', output)
    print('Original ego path:',cfg['path_file'])
    print('Mission indices:',m)
    print('Circulating conflict:',[cl,ch])
    print('Approach / merge candidates (m/s):',cfg['approach_speed'],
          (cfg['merge_speed'],min(merge_ceiling,cfg['test_speed_limit'])))
    print('Geometry is an initial map-based estimate, not a simulator validation.')


if __name__=='__main__':
    import rospkg
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--existing',required=True)
    p.add_argument('--output',required=True)
    a=p.parse_args()
    prepare(a.existing,a.output,Path(rospkg.RosPack().get_path('roundabout_mission'))/'config/roundabout.yaml')
