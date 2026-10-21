#!/usr/bin/env bash
# Run INSIDE the already-running Noetic container, after extracting this package.
set -eo pipefail
WS=/root/morai_ws
PKG="$WS/src/ROI/src/control/roundabout_mission"
ORIGINAL_CONFIG="${1:-/root/roundabout_mission.yaml}"
test -f "$ORIGINAL_CONFIG"
source /opt/ros/noetic/setup.bash
cd "$WS"
catkin_make --pkg roundabout_mission
source "$WS/devel/setup.bash"
rospack profile >/dev/null
RESOLVED_PACKAGE="$(rospack find roundabout_mission)"
if [ "$(readlink -f "$RESOLVED_PACKAGE")" != "$(readlink -f "$PKG")" ]; then
  echo "ERROR: ROS resolves another package: $RESOLVED_PACKAGE" >&2
  exit 1
fi
python3 - <<'PY'
import os
import inspect
from roundabout_mission import VERSION
from roundabout_mission import core
from purepursuit_mgeo.path import MgeoPurePursuit
from path_planning.longitudinal_controller import PedalSpeedController
assert VERSION == '4.1.0', (VERSION,core.__file__)
assert os.path.realpath(core.__file__).startswith('/root/morai_ws/src/ROI/src/control/roundabout_mission/'),core.__file__
inspect.signature(MgeoPurePursuit).bind([],3.,4.,.35,1.5,1.)
inspect.signature(MgeoPurePursuit.compute).bind(None,0.,0.,0.,0.)
inspect.signature(PedalSpeedController).bind(kp=.075,ki=.0001,kd=.025,nominal_dt=.05,max_accel=.5,max_brake=1.)
print('VERIFIED',VERSION,os.path.realpath(core.__file__))
PY
CONFIG_OUTPUT="/root/roundabout_mission_v4_$(date +%Y%m%d_%H%M%S).yaml"
rosrun roundabout_mission prepare_roundabout_v4.py --existing "$ORIGINAL_CONFIG" --output "$CONFIG_OUTPUT"
cp "$CONFIG_OUTPUT" /root/roundabout_mission_v4.yaml
printf '%s\n' 'Installed v4; config: /root/roundabout_mission_v4.yaml' 'Stop other driving launches before starting the v4 standalone launch.'
