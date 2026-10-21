import copy
import importlib.util
import json
import math
from pathlib import Path
import threading
import types
import unittest
from unittest.mock import patch
from roundabout_mission.core import Decision,Ego,Track,swept_boxes
from roundabout_mission.integration import apply_mission
from roundabout_mission.ros_io import fresh
from roundabout_mission import VERSION
from test_core import CFG,PATH,CIRC
from test_driver_adapter import DriverAdapter,Publisher


class GeometryAndScope(unittest.TestCase):
    def test_normal_route_does_not_wait_for_roundabout_lidar(self):
        d=Decision(PATH,CIRC,copy.deepcopy(CFG))
        r=d.step(Ego(0,0,0,0),1,True,False)
        self.assertFalse(r['active'])
        self.assertFalse(r['stop_required'])
        self.assertEqual(r['state'],'CRUISE')
        self.assertEqual(apply_mission(2.,r,True)[:2],(2.,False))

    def test_lidar_watchdog_at_approach(self):
        d=Decision(PATH,CIRC,copy.deepcopy(CFG))
        r=d.step(Ego(5,0,0,0),1,True,False)
        self.assertTrue(r['emergency'])
        self.assertEqual(r['decision_reason'],'obstacles_stale_or_invalid')

    def test_user_roadside_fragments_with_north_heading(self):
        e=Ego(-99.3629613579,307.7765382008,math.pi/2,0)
        tracks=[Track(3,-101.6038,309.2513,-3.10045,.28310,.01991,-.00426,.06453),
                Track(4,-101.5906,307.4139,-.01179,.25845,.01709,.01763,-.83168),
                Track(16,-101.7170,311.8856,-3.11498,.27972,.01917,0,0),
                Track(17,-101.7731,314.8784,1.56659,3.35132,.44565,0,0)]
        for t in tracks:
            with self.subTest(id=t.id):
                sep,ttc=swept_boxes(e,t,CFG)
                self.assertGreater(sep,0)
                self.assertTrue(ttc is None or ttc>CFG['emergency_ttc'])

    def test_sideways_long_vehicle_is_not_excluded_by_width(self):
        # V2's lateral gate incorrectly used only width for a rotated vehicle.
        e=Ego(0,0,0,0)
        t=Track(8,1.3,2.0,math.pi/2,4.5,1.8,0,0)
        self.assertEqual(swept_boxes(e,t,CFG)[1],0)

    def test_crossing_vehicle_ttc(self):
        e=Ego(0,0,0,0)
        t=Track(8,1.3,6.,math.pi/2,4.5,1.8,0,-5.)
        self.assertLess(swept_boxes(e,t,CFG)[1],1.)

    def test_stationary_obstacle_on_route_is_retained(self):
        d=Decision(PATH,CIRC,copy.deepcopy(CFG))
        d.update_tracks([Track(7,9,0,0,.3,.02,0,0)],1.)
        self.assertTrue(d.step(Ego(7,0,0,1),1.)['emergency'])

    def test_mission_never_increases_teammates_speed_cap(self):
        r=dict(active=True,target_speed=3.,stop_required=False,decision_reason='follow')
        self.assertEqual(apply_mission(1.,r,True)[0],1.)
        self.assertTrue(apply_mission(1.,r,False)[1])

    def test_slow_merge_checks_traffic_beyond_nominal_horizon(self):
        cfg=copy.deepcopy(CFG);cfg['merge_speed']=.5
        d=Decision(PATH,CIRC,cfg)
        d.update_tracks([Track(8,32,-25,math.pi/2,4.5,1.8,0,1)],1.)
        r=d.step(Ego(15.9,0,0,0),1.)
        self.assertEqual(r['blocking_vehicle_id'],8)
        self.assertTrue(r['stop_required'])

    def test_static_conflict_does_not_expire_at_prediction_horizon(self):
        cfg=copy.deepcopy(CFG);cfg['merge_speed']=.5
        d=Decision(PATH,CIRC,cfg)
        d.update_tracks([Track(8,32,4,math.pi/2,4.5,1.8,0,0)],1.)
        r=d.step(Ego(15.9,0,0,0),1.)
        self.assertEqual(r['blocking_vehicle_id'],8)
        self.assertTrue(r['stop_required'])


class DriverRegression(DriverAdapter):
    def test_duplicate_odom_does_not_replace_pose_or_refresh_watchdog(self):
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda:10.)))
        old=self.d.ego
        with patch.object(self.module,'parse_odom',return_value=Ego(999,999,0,0)):
            self.d.odom(msg)
        self.assertIs(self.d.ego,old)
        self.assertEqual(self.d.odom_wall,100.)
        self.assertFalse(fresh(self.d.odom_stamp,self.d.odom_wall,11.,101.,.5))

    def test_cruise_positive_accel_even_with_inactive_zero_mission_target(self):
        self.d.ego.speed=0
        self.d.status.update(active=False,target_speed=0,decision_reason='csv_cruise')
        self.assertGreater(self.tick().accel,0)

    def test_diagnostics_identify_emergency_stop(self):
        self.d.status.update(stop_required=True,emergency=True,decision_reason='emergency_collision_risk')
        self.tick()
        row=json.loads(self.d.diagnostics.messages[-1].data)
        self.assertEqual(row['control_reason'],'emergency_collision_risk')
        self.assertEqual(row['version'],VERSION)

class GateTimestampRegression(unittest.TestCase):
    def test_repeated_packet_preserves_sample_without_extending_age(self):
        import sys
        modules={n:types.ModuleType(n) for n in ('rospy','nav_msgs','nav_msgs.msg','std_msgs','std_msgs.msg','lidar_perception','lidar_perception.msg')}
        modules['nav_msgs.msg'].Odometry=object
        for name in ('Bool','Float64','String'):setattr(modules['std_msgs.msg'],name,object)
        modules['lidar_perception.msg'].LidarObstacleArray=object
        modules['rospy'].Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_sec=lambda:10.1))
        modules['rospy'].logwarn_throttle=lambda *a:None
        source=Path(__file__).parents[1]/'scripts/roundabout_merge_gate.py'
        with patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('gate_timestamp_test',source)
            m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        g=m.Gate.__new__(m.Gate)
        g.lock=threading.RLock();g.frame='map';g.cfg=CFG
        g.stamps={'odom':10.};g.received={'odom':100.};g.ego=Ego(1,2,0,0)
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda:10.)))
        g.receive(msg,'odom',lambda *a:Ego(99,99,0,0))
        self.assertEqual(g.ego.x,1)
        self.assertEqual(g.stamps['odom'],10.)
        self.assertEqual(g.received['odom'],100.)
        self.assertFalse(fresh(g.stamps['odom'],g.received['odom'],10.7,100.7,.5))
