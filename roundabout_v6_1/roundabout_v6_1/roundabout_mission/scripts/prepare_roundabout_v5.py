#!/usr/bin/env python3
"""Copy a working v4.2 mission config without recomputing routes or tuning."""
import argparse
from pathlib import Path
import yaml
from purepursuit_mgeo.path import load_mgeo_path
from roundabout_mission.core import Decision

def prepare(existing, output):
    old=yaml.safe_load(Path(existing).read_text())
    cfg=dict(old)
    for key,value in dict(entry_occlusion_hold_s=.8,entry_vehicle_length_m=4.5,entry_vehicle_width_m=1.8).items():
        cfg.setdefault(key,value)
    Decision(load_mgeo_path(cfg['path_file']),load_mgeo_path(cfg['circulating_path_file']),cfg)
    with open(output,'x') as stream:yaml.safe_dump(cfg,stream,sort_keys=False)
    print('Created',output,'; existing route, mission geometry and tuning preserved.')
    for key in cfg:
        if key not in old:print('Added',key,cfg[key])

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--existing',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();prepare(args.existing,args.output)
