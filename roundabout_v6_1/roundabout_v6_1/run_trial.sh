#!/usr/bin/env bash
set -eo pipefail
bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
test -f /opt/ros/noetic/setup.bash || { echo 'Run inside the existing ROS Noetic container.' >&2; exit 1; }
source /opt/ros/noetic/setup.bash
source /root/morai_ws/devel/setup.bash
python3 - "$bundle_dir/v61_speedtest.yaml" <<'PY'
import sys, yaml
from roundabout_mission import VERSION
if VERSION != '6.1.0':
    raise SystemExit('Installed version is '+VERSION+'. Run this bundle/install.sh first.')
c=yaml.safe_load(open(sys.argv[1]))
assert c['minimum_gap'] == 1.0, c['minimum_gap']
print('CODE_VERSION='+VERSION+' minimum_gap='+str(c['minimum_gap'])+' time_headway='+str(c['time_headway']))
PY
exec rosrun roundabout_mission roundabout_record_trial.py \
  --config "$bundle_dir/v61_speedtest.yaml" \
  --morai-host-ip "${ROUNDABOUT_MORAI_IP:-192.168.0.147}" \
  --profile baseline \
  --output-root "${ROUNDABOUT_TRIAL_ROOT:-${ROUNDABOUT_LOG_ROOT:-/root/roundabout_trials}}" \
  --note "v6_1_0_trial" \
  --launch-arg rviz:=true "$@"
