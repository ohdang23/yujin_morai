"""Validation shared by the two ROS adapters; does not import rospy."""
import csv
import json
import math
import os
import time
from .core import Ego, Track


def fresh(stamp, received, now, wall, timeout):
    return stamp is not None and 0 <= now-stamp <= timeout and 0 <= wall-received <= timeout


def parse_odom(msg, frame):
    p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
    values = (p.x,p.y,q.x,q.y,q.z,q.w,v.x,v.y)
    if msg.header.frame_id != frame or not all(math.isfinite(x) for x in values):
        raise ValueError('invalid odometry frame or values')
    if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .05:
        raise ValueError('invalid orientation quaternion')
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return Ego(p.x,p.y,yaw,math.hypot(v.x,v.y))


def parse_tracks(msg, frame):
    if msg.header.frame_id != frame or msg.obstacle_count != len(msg.obstacles):
        raise ValueError('invalid obstacle frame/count')
    result = []
    ids = set()
    for o in msg.obstacles:
        values = (o.center_x_map,o.center_y_map,o.yaw,o.length,o.width,o.velocity_x_map,o.velocity_y_map)
        if not all(math.isfinite(x) for x in values) or o.length <= 0 or o.width <= 0 or o.id in ids:
            raise ValueError('invalid obstacle fields/duplicate ID')
        ids.add(o.id)
        result.append(Track(o.id,*values))
    return result


class CsvLog:
    def __init__(self, directory, prefix):
        self.stream = self.writer = self.json_stream = None
        if directory:
            os.makedirs(os.path.expanduser(directory), exist_ok=True)
            path = os.path.join(os.path.expanduser(directory), prefix+'_'+str(time.time_ns())+'.csv')
            self.stream = open(path, 'w', newline='', encoding='utf-8')
            self.json_stream = open(path[:-4]+'.jsonl', 'w', encoding='utf-8')

    def write(self, row):
        if self.stream:
            full = dict(row, log_wall_time_s=time.time(), log_monotonic_s=time.monotonic())
            self.json_stream.write(json.dumps(full, ensure_ascii=False, allow_nan=False)+'\n')
            self.json_stream.flush()
            if self.writer is None:
                fields = 'timestamp state ego_x ego_y ego_yaw_rad ego_speed ego_s active target_speed actual_target_speed accel_cmd brake_cmd steering_cmd stop_required allowed emergency blocking_vehicle_id vehicle_eta ego_eta merge_gap lead_vehicle_id lead_x lead_y lead_speed lead_gap desired_gap relative_speed time_gap minimum_ttc minimum_distance decision_reason driver_input_fresh control_enabled lost_track_ids version odom_fresh obstacle_fresh emergency_track_ids interval_blocking_ids predicted_blocking_ids predicted_collision_time predicted_collisions prediction_complete prediction_model prediction_horizon_s control_reason command_topic target_index recovered_track_ids id_handoffs recovery_frames recovery_span_s track_recovery_reason decision_compute_ms decision_phase gap_check_active stop_line_distance_m yield_base_s safe_frames safe_confirm_elapsed_s blocking_track_details selected_merge_speed_mps merge_speed_candidates dangerous_vehicle_count conflict_candidate_count minimum_conflict_eta ego_conflict_eta minimum_time_gap safe_duration commit_latched emergency_active'.split()
                fields += 'decision_reason_detail lead_relative_following_active lead_prediction_stop_suppressed lead_close_gap_stop_suppressed lead_prediction_suppressed_ids lead_gap_control_mode lead_current_separation_m lead_closing_speed_mps lead_relative_speed_cap_mps'.split()
                fields += 'log_wall_time_s log_monotonic_s'.split()
                self.writer = csv.DictWriter(self.stream, fieldnames=fields, extrasaction='ignore')
                self.writer.writeheader()
            self.writer.writerow({k: json.dumps(v, ensure_ascii=False, allow_nan=False)
                if isinstance(v, (dict, list, tuple)) else v for k,v in full.items()})
            self.stream.flush()

    def close(self):
        if self.stream:
            self.stream.close()
        if self.json_stream:
            self.json_stream.close()
