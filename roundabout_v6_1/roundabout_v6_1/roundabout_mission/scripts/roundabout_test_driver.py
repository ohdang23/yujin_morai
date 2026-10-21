#!/usr/bin/env python3
"""Standalone test controller using the repository's path, Pure Pursuit and PID.

Consumes the atomic timestamped status, avoiding inconsistent Bool/speed cycles.
"""
import json
import math
import threading
import time
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from morai_msgs.msg import CtrlCmd
from purepursuit_mgeo.path import MgeoPurePursuit, load_mgeo_path
from path_planning.longitudinal_controller import PedalSpeedController
from roundabout_mission.ros_io import fresh, parse_odom, CsvLog
from roundabout_mission.core import Polyline
from roundabout_mission.integration import apply_mission
from roundabout_mission import VERSION


class Driver:
    def __init__(self):
        self.c = c = rospy.get_param('~')
        self.lock = threading.RLock()
        points = load_mgeo_path(c['path_file'])
        self.route = Polyline(points)
        self.pp = MgeoPurePursuit(points,c['wheelbase_m'],c['lookahead_min_m'],c['lookahead_gain'],c['goal_tolerance_m'],c['steering_sign'])
        self.pid = PedalSpeedController(kp=c['speed_kp'],ki=c['speed_ki'],kd=c['speed_kd'],nominal_dt=1/c['rate_hz'],max_accel=c['max_accel_pedal'],max_brake=c['max_brake_pedal'])
        self.ego = self.status = None
        self.odom_stamp = self.status_stamp = None
        self.odom_wall = self.status_wall = 0.
        self.accel = self.brake = 0.
        self.last_wall = time.monotonic()
        self.log = CsvLog(c.get('log_dir',''),'control')
        self.pub = rospy.Publisher(c.get('command_topic','/ctrl_cmd'),CtrlCmd,queue_size=1) if c['enable_control'] else None
        rospy.loginfo('Roundabout driver: enable_control=%s command_topic=%s path=%s',
                      c['enable_control'], c.get('command_topic','/ctrl_cmd'), c['path_file'])
        self.diagnostics = rospy.Publisher('~status', String, queue_size=1)
        self.preview = rospy.Publisher('~command_preview',CtrlCmd,queue_size=1)
        rospy.Subscriber(c.get('odom_topic','/localization/odometry'),Odometry,self.odom,queue_size=1)
        rospy.Subscriber('/roundabout_merge_gate/status',String,self.decision,queue_size=1)
        rospy.on_shutdown(self.shutdown)
        self.timer = rospy.Timer(rospy.Duration(1/c['rate_hz']),self.tick)

    def odom(self,msg):
        with self.lock:
            try:
                ego = parse_odom(msg,self.c['map_frame'])
                stamp = msg.header.stamp.to_sec()
                if not math.isfinite(stamp) or stamp <= 0 or stamp > rospy.Time.now().to_sec()+.02:
                    raise ValueError('out of order odometry')
                if self.odom_stamp is not None and stamp <= self.odom_stamp:
                    rospy.logwarn_throttle(5., 'repeated/out-of-order odometry; keeping last valid sample')
                    return
                self.ego = ego
                self.odom_stamp,self.odom_wall = stamp,time.monotonic()
            except ValueError:
                # Keep the previous valid odometry until its normal timeout.
                # A single duplicate packet must not trigger a full brake.
                pass

    def decision(self,msg):
        with self.lock:
            try:
                data = json.loads(msg.data)
                if (not isinstance(data,dict) or
                        not all(type(data[k]) in (int,float) and math.isfinite(data[k]) for k in ('timestamp','target_speed')) or
                        data['target_speed'] < 0 or
                        not all(type(data[k]) is bool for k in ('active','stop_required','emergency')) or
                        not isinstance(data['decision_reason'],str)):
                    raise ValueError('invalid status')
                if self.status_stamp is not None and data['timestamp'] <= self.status_stamp:
                    rospy.logwarn_throttle(5., 'repeated/out-of-order decision status; keeping last valid sample')
                    return
                if data['timestamp'] <= 0 or data['timestamp'] > rospy.Time.now().to_sec()+.02:
                    raise ValueError('invalid decision timestamp')
                self.status = data
                self.status_stamp,self.status_wall = data['timestamp'],time.monotonic()
            except (ValueError,KeyError,TypeError):
                # Keep the previous valid decision until decision_timeout_s.
                pass

    def tick(self,event):
        with self.lock:
            c = self.c
            now,wall = rospy.Time.now().to_sec(),time.monotonic()
            dt = max(0.,min(.2,wall-self.last_wall))
            self.last_wall = wall
            valid = fresh(self.odom_stamp,self.odom_wall,now,wall,c['odom_timeout_s']) and fresh(self.status_stamp,self.status_wall,now,wall,c['decision_timeout_s'])
            steering,goal = 0.,False
            target = 0.
            hard_stop = not valid
            stop = True
            reason = 'driver_input_stale'
            goal_index = None
            if valid:
                e = self.ego
                steering,goal,_,goal_index,_ = self.pp.compute(e.x,e.y,e.yaw,e.speed)
                # Normal cruise and active mission limits are separate. During
                # the mission, honor the gate's current speed request: it may
                # be zero while waiting, or a merge speed above approach
                # cruise after a gap is authorized. The configured test limit
                # remains the hard ceiling.
                requested = float(self.status.get('target_speed',0.))
                base_target = min(requested,c['test_speed_limit']) if self.status.get('active',False) else min(c.get('cruise_speed',c['test_speed_limit']),c['test_speed_limit'])
                curvature = abs(math.tan(steering)/c['wheelbase_m'])
                base_target = min(base_target, math.sqrt(c.get('max_lateral_accel',1.5)/max(curvature,1e-9)))
                target, mission_stop, reason = apply_mission(base_target, self.status, True)
                # A route endpoint is a stop, regardless of decision state.
                distance = self.route.s[-1]-self.route.project(e.x,e.y)[1]
                target = min(target,math.sqrt(2*c['comfortable_decel']*max(0.,distance-c['goal_tolerance_m'])))
                stop = mission_stop or goal
                if goal:
                    reason = 'route_goal'
                hard_stop = self.status.get('emergency',False)
                if stop:
                    target = 0.
            if hard_stop:
                self.pid.reset()
                self.accel,self.brake = 0.,1.
            elif stop and self.ego.speed < .15:
                self.pid.reset()
                self.accel,self.brake = 0.,c['hold_brake']
            else:
                accel,brake = self.pid.compute(target,self.ego.speed,now)
                # Never prolong throttle when a stop or speed reduction is requested.
                if stop or target < self.ego.speed:
                    accel = 0.
                self.accel = min(accel,self.accel+c['max_accel_rate']*dt) if brake <= 0 else 0.
                self.brake = min(brake,self.brake+c['max_brake_rate']*dt) if self.accel <= 0 else 0.
            cmd = CtrlCmd()
            cmd.longlCmdType = 1
            cmd.steering = max(-c['max_steering_rad'],min(c['max_steering_rad'],steering))
            cmd.accel,cmd.brake = self.accel,self.brake
            cmd.velocity,cmd.acceleration = 0.,0.
            self.preview.publish(cmd)
            if self.pub:
                self.pub.publish(cmd)
            row = dict(self.status or {})
            row.update(timestamp=now,actual_target_speed=target,accel_cmd=cmd.accel,brake_cmd=cmd.brake,steering_cmd=cmd.steering,driver_input_fresh=valid,control_enabled=c['enable_control'])
            row.update(version=VERSION, control_reason=reason, command_topic=c.get('command_topic','/ctrl_cmd'),
                       target_index=goal_index)
            self.diagnostics.publish(String(data=json.dumps(row,allow_nan=False)))
            rospy.loginfo_throttle(2., 'Driver %s reason=%s target=%.2f accel=%.3f brake=%.3f enabled=%s',
                                  VERSION, reason, target, cmd.accel, cmd.brake, c['enable_control'])
            self.log.write(row)

    def shutdown(self):
        if self.pub:
            cmd = CtrlCmd()
            cmd.longlCmdType,cmd.brake = 1,1.
            self.pub.publish(cmd)
        self.log.close()


if __name__ == '__main__':
    rospy.init_node('roundabout_test_driver')
    Driver()
    rospy.spin()
