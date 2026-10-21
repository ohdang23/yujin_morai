#!/usr/bin/env bash
set -euo pipefail
bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
target_dir="${1:-/root/morai_ws/src/ROI/src/control/roundabout_mission}"
if [[ ! -f "$target_dir/package.xml" ]]; then
  echo "Existing roundabout_mission package not found: $target_dir" >&2
  exit 1
fi
python3 -m compileall -q "$bundle_dir/roundabout_mission/src" "$bundle_dir/roundabout_mission/scripts"
backup_root="${ROUNDABOUT_BACKUP_ROOT:-$HOME/roundabout_backups}"
backup_dir="$backup_root/before_v61_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$backup_dir"
tar -czf "$backup_dir/roundabout_mission.tar.gz" -C "$(dirname -- "$target_dir")" "$(basename -- "$target_dir")"
# Replace the complete package, so old V5.3/V5.4 Python modules cannot mix in.
cp -a "$bundle_dir/roundabout_mission" "$backup_dir/new_package"
find "$backup_dir/new_package" -type d -name __pycache__ -prune -exec rm -rf {} +
chmod +x "$backup_dir/new_package"/scripts/*.py
mv "$target_dir" "$backup_dir/previous_package"
if ! mv "$backup_dir/new_package" "$target_dir"; then
  mv "$backup_dir/previous_package" "$target_dir"
  exit 1
fi
printf 'Installed 6.1.0: %s\nBackup: %s\n' "$target_dir" "$backup_dir/roundabout_mission.tar.gz"
printf 'Run this bundle/run_trial.sh inside the ROS Noetic container.\n'
