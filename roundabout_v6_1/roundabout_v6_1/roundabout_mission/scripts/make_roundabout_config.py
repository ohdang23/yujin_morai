#!/usr/bin/env python3
"""Print point index table, or generate a validated mission YAML from selections."""
import argparse
import csv
from pathlib import Path
import yaml
from purepursuit_mgeo.path import load_mgeo_path
from roundabout_mission.core import Decision, Polyline

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--path',required=True)
p.add_argument('--inspect',action='store_true',help='Print every loaded point index,x,y,s; redirect to CSV')
p.add_argument('--circulating-path')
p.add_argument('--indices',type=int,nargs=7,metavar=('APPROACH','STOP','ENTRY','CONFLICT_IN','CONFLICT_OUT','MERGED','EXIT'))
p.add_argument('--circulating-conflict',type=int,nargs=2)
p.add_argument('--template')
p.add_argument('--output')
a = p.parse_args()
points = load_mgeo_path(a.path)
if a.inspect:
    import sys
    writer = csv.writer(sys.stdout)
    writer.writerow(['index','x_map','y_map','s_m'])
    writer.writerows((i,t.x,t.y,s) for i,(t,s) in enumerate(zip(points,Polyline(points).s)))
else:
    if not all((a.circulating_path,a.indices,a.circulating_conflict,a.template,a.output)):
        p.error('generation needs --circulating-path --indices --circulating-conflict --template --output')
    with open(a.template) as f:
        c = yaml.safe_load(f)
    c['path_file'] = str(Path(a.path).resolve())
    c['circulating_path_file'] = str(Path(a.circulating_path).resolve())
    c['mission_indices'] = dict(zip(['approach','stop_line','entry','conflict_start','conflict_end','merge_complete','exit'],a.indices))
    c['circulating_conflict_indices'] = a.circulating_conflict
    circulating = load_mgeo_path(a.circulating_path)
    c['roi_map'] = [min(t.x for t in circulating)-5,max(t.x for t in circulating)+5,min(t.y for t in circulating)-5,max(t.y for t in circulating)+5]
    Decision(points,circulating,c)
    with open(a.output,'x') as f:
        yaml.safe_dump(c,f,sort_keys=False)
    print('Created '+a.output+'; verify both conflict intervals describe the SAME physical zone.')
