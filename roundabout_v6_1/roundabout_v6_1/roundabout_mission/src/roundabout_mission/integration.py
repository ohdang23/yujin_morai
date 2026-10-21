"""Contract for the team's controller: compose BEFORE its pedal PID.

Use the timestamped JSON status, with both ROS-time and wall-time watchdogs.
An inactive gate adds no speed cap; active gate caps the existing planner.
Never use allowed=False as a global stop (it is false during normal cruise).
"""
def apply_mission(base_target, status, status_fresh):
    if not status_fresh or status is None:
        return 0., True, 'decision_stale'
    if status['stop_required'] or status.get('emergency', False):
        return 0., True, status['decision_reason']
    if status.get('active', False):
        return min(base_target, status['target_speed']), False, status['decision_reason']
    return base_target, False, 'csv_cruise'
