#!/usr/bin/env python3
"""Create a V6-series copy of a working YAML with a 1 m base following gap."""
import argparse
from pathlib import Path
import yaml
from purepursuit_mgeo.path import load_mgeo_path
from roundabout_mission import VERSION
from roundabout_mission.core import Decision

def prepare(existing, output):
    cfg=dict(yaml.safe_load(Path(existing).read_text()))
    for key,value in dict(entry_occlusion_hold_s=.8,entry_vehicle_length_m=4.5,entry_vehicle_width_m=1.8).items():
        cfg.setdefault(key,value)
    cfg.setdefault('static_minimum_gap_m',cfg['minimum_gap'])
    cfg['minimum_gap']=1.0
    cfg['version']=VERSION
    if VERSION == '6.1.0':
        cfg.update(lead_relative_following_enabled=True,
                   lead_follow_alignment_cos_min=0.9,
                   lead_follow_lateral_tolerance_m=0.9,
                   lead_relative_reaction_s=0.3)
    Decision(load_mgeo_path(cfg['path_file']),load_mgeo_path(cfg['circulating_path_file']),cfg)
    with open(output,'x') as stream:
        yaml.safe_dump(cfg,stream,sort_keys=False)
    print('Created',output,'version='+VERSION,'minimum_gap=1.0')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--existing',required=True)
    p.add_argument('--output',required=True)
    args=p.parse_args()
    prepare(args.existing,args.output)
