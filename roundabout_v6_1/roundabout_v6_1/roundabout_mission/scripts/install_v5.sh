#!/usr/bin/env bash
# Run after replacing the package with the extracted v5 directory.
set -eo pipefail
ORIGINAL_CONFIG="${1:?Pass the working v4.2 mission YAML as argument 1}"
OUTPUT_CONFIG="${2:-/root/roundabout_mission_v5.yaml}"
test -f "$ORIGINAL_CONFIG"
if [ -e "$OUTPUT_CONFIG" ]; then
  echo "Output already exists; choose a new filename: $OUTPUT_CONFIG" >&2
  exit 1
fi
source /opt/ros/noetic/setup.bash
cd /root/morai_ws
catkin_make --pkg roundabout_mission
source devel/setup.bash
python3 -c 'from roundabout_mission import VERSION; assert VERSION == "5.0.0", VERSION'
rosrun roundabout_mission prepare_roundabout_v5.py --existing "$ORIGINAL_CONFIG" --output "$OUTPUT_CONFIG"
printf 'v5 config: %s\n' "$OUTPUT_CONFIG"
