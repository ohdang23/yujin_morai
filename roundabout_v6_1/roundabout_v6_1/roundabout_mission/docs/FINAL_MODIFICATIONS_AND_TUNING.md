# v4.2 final local modifications

This package is intended for MORAI roundabout simulation testing.

## Logic changes
- Stationary tracks (`speed < min_vehicle_speed`) are excluded from merge trajectory blockers.
- Stationary tracks are excluded from lead-vehicle following.
- Track-loss recovery also runs inside the conflict area, avoiding a permanent stop on an ID loss/handoff.
- After a safe gap is confirmed and `COMMIT` starts, a new non-emergency gap blocker does not bounce the state back to `WAIT`. Emergency collision checks remain active.

## Final tuned values
- `track_hold_s: 0.4`
- `safe_confirm_s: 0.3`
- `front_time_buffer_s: 0.8`
- `rear_time_buffer_s: 1.2`
- `lost_recheck_s: 0.3`
- `lost_recheck_frames: 2`
- `reid_max_age_s: 2.5`
- `reid_distance_m: 3.0`
- `reid_velocity_delta_mps: 3.0`
- `merge_speed: 5.0 m/s`
- `roundabout_speed: 5.0 m/s`
- `minimum_merge_speed: 5.0 m/s`
- `maximum_merge_speed: 5.0 m/s`
- `test_speed_limit: 5.0 m/s`
- `max_accel: 2.5 m/s^2`
- `speed_kp: 0.30`
- `max_accel_pedal: 1.0`
- `max_accel_rate: 3.0`

## Important experiment parameters
1. `merge_speed`, `roundabout_speed`: commanded merge/circulating speed.
2. `max_accel`, `speed_kp`, `max_accel_pedal`, `max_accel_rate`: launch response.
3. `front_time_buffer_s`, `rear_time_buffer_s`, `safe_confirm_s`: gap acceptance conservatism.
4. `min_vehicle_speed`: stationary/moving classification threshold.
5. `reid_*`, `lost_recheck_*`, `track_hold_s`: tracker-ID loss/handoff robustness.
6. `emergency_ttc`, `emergency_gap_m`: emergency safety thresholds; tune cautiously.

The generated runtime config (`/root/roundabout_mission_v42.yaml`) is created by `install_v42.sh` from the existing configured v4/v4.1 YAML, preserving route paths and mission indices while applying the tuned values above.
