#!/usr/bin/env python3
"""LiDAR/odometry adapter; publishes decision only, never pedals."""
import json
import math
import threading
import time
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String
from lidar_perception.msg import LidarObstacleArray
from purepursuit_mgeo.path import load_mgeo_path
from roundabout_mission.core import Decision, Ego
from roundabout_mission import VERSION
from roundabout_mission.ros_io import fresh, parse_odom, parse_tracks, CsvLog


class Gate:
    def __init__(self):
        cfg = rospy.get_param('~')
        self.engine = Decision(load_mgeo_path(cfg['path_file']), load_mgeo_path(cfg['circulating_path_file']),cfg)
        rospy.loginfo('Roundabout %s core=%s', VERSION, __import__('roundabout_mission.core', fromlist=['x']).__file__)
        self.cfg, self.lock = cfg, threading.RLock()
        self.ego = Ego(0.,0.,0.,0.)
        self.stamps, self.received = {}, {}
        self.frame = cfg.get('map_frame','map')
        self.log = CsvLog(cfg.get('log_dir',''), 'decision')
        self.stop = rospy.Publisher('~stop_required', Bool, queue_size=1)
        self.speed = rospy.Publisher('~target_speed', Float64, queue_size=1)
        self.allowed = rospy.Publisher('~allowed', Bool, queue_size=1)
        from visualization_msgs.msg import MarkerArray
        self.gap_markers = rospy.Publisher('~gap_markers', MarkerArray, queue_size=1)
        self.status = rospy.Publisher('~status', String, queue_size=1)
        rospy.Subscriber(cfg.get('odom_topic','/localization/odometry'),Odometry,self.odom,queue_size=1)
        rospy.Subscriber(cfg.get('obstacle_topic','/perception/lidar/tracked_obstacles_map'),LidarObstacleArray,self.obstacles,queue_size=1)
        rospy.on_shutdown(self.log.close)
        self.timer = rospy.Timer(rospy.Duration(1./cfg['rate_hz']),self.tick)

    def receive(self, msg, key, parse):
        with self.lock:
            try:
                value = parse(msg,self.frame)
                stamp = msg.header.stamp.to_sec()
                now = rospy.Time.now().to_sec()
                if not math.isfinite(stamp) or stamp <= 0 or stamp > now+.02:
                    raise ValueError('invalid/repeated/out-of-order timestamp')
                # MORAI/ROS adapters can publish the same source timestamp more
                # than once.  A duplicate is not a sensor failure: keep the
                # last accepted sample and let fresh() age it out normally.
                if key in self.stamps and self.stamps[key] is not None and stamp <= self.stamps[key]:
                    rospy.logwarn_throttle(5., '%s repeated/out-of-order timestamp; keeping last valid sample', key)
                    return
                if now-stamp > self.cfg[key+'_timeout_s']:
                    raise ValueError('stale timestamp')
                if key == 'odom':
                    self.ego = value
                else:
                    self.engine.update_tracks(value,stamp)
                self.stamps[key],self.received[key] = stamp,time.monotonic()
            except (ValueError,TypeError) as exc:
                # Do not erase a recent valid sample because one malformed or
                # delayed packet arrived.  fresh() will force a safe stop if
                # valid data actually stops arriving for the configured timeout.
                rospy.logwarn_throttle(2.,str(exc))

    def odom(self,msg):
        self.receive(msg,'odom',parse_odom)

    def obstacles(self,msg):
        self.receive(msg,'obstacle',parse_tracks)

    def tick(self,event):
        with self.lock:
            now,wall = rospy.Time.now().to_sec(),time.monotonic()
            validity = {k: fresh(self.stamps.get(k), self.received.get(k,0), now, wall, self.cfg[k+'_timeout_s']) for k in ('odom','obstacle')}
            row = self.engine.step(self.ego, now, validity['odom'], validity['obstacle'])
            row['decision_compute_ms']=(time.monotonic()-wall)*1000.
            row.update(version=VERSION, odom_fresh=validity['odom'], obstacle_fresh=validity['obstacle'])
            rospy.loginfo_throttle(2., 'Mission %s active=%s reason=%s target=%.2f stop=%s ids=%s',
                                  row['state'], row['active'], row['decision_reason'], row['target_speed'],
                                  row['stop_required'], row.get('emergency_track_ids', []))
            self.stop.publish(Bool(data=row['stop_required']))
            self.speed.publish(Float64(data=row['target_speed']))
            self.allowed.publish(Bool(data=row['allowed']))
            self.status.publish(String(data=json.dumps(row,allow_nan=False)))
            from roundabout_mission.visualization import markers
            self.gap_markers.publish(markers(self.engine,row,self.frame,rospy.Time.now()))
            self.log.write(row)


if __name__ == '__main__':
    rospy.init_node('roundabout_merge_gate')
    Gate()
    rospy.spin()
