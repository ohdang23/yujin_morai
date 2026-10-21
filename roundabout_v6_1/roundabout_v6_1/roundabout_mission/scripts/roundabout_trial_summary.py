#!/usr/bin/env python3
"""Summarize and optionally repackage one trial. Works without a running ROS master."""
import argparse
import json
from roundabout_mission.trial_tools import summarize, package_trial


def main():
    p=argparse.ArgumentParser()
    p.add_argument('directory')
    p.add_argument('--pack', action='store_true')
    a=p.parse_args()
    result=summarize(a.directory)
    print(json.dumps({k:v for k,v in result.items() if k!='transitions'},ensure_ascii=False,indent=2))
    if a.pack:
        print('SEND_THIS_FILE='+str(package_trial(a.directory)))


if __name__=='__main__':
    main()
