"""Exercise the actual driver with ROS transport doubles (not a ROS runtime)."""
import importlib.util
import sys
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import patch
from roundabout_mission.core import Ego, Polyline
from purepursuit_mgeo.path import MgeoPurePursuit, PathPoint
from path_planning.longitudinal_controller import PedalSpeedController
from test_core import CFG


class Publisher:
    def __init__(self): self.messages=[]
    def publish(self,msg): self.messages.append(msg)


class Log:
    def write(self,row): self.row=row


class DriverAdapter(unittest.TestCase):
    def setUp(self):
        modules={}
        for name in ['rospy','nav_msgs','nav_msgs.msg','std_msgs','std_msgs.msg','morai_msgs','morai_msgs.msg']:
            modules[name]=types.ModuleType(name)
        modules['nav_msgs.msg'].Odometry=object
        modules['std_msgs.msg'].String=lambda **kw:types.SimpleNamespace(**kw)
        modules['rospy'].loginfo_throttle=lambda *a:None
        modules['rospy'].logwarn_throttle=lambda *a:None
        modules['morai_msgs.msg'].CtrlCmd=type('CtrlCmd',(),{})
        modules['rospy'].Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_sec=lambda:10.))
        source=Path(__file__).parents[1]/'scripts/roundabout_test_driver.py'
        with patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('driver_under_test',source)
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.module=module
        d=self.d=module.Driver.__new__(module.Driver)
        d.c=dict(CFG,enable_control=True)
        d.lock=threading.RLock()
        d.pp=MgeoPurePursuit([PathPoint(x,0,0) for x in range(101)],3,4,.35,1.5)
        d.route=Polyline(d.pp.points)
        d.diagnostics=Publisher()
        d.pid=PedalSpeedController()
        d.ego=Ego(20,0,0,1)
        d.status=dict(timestamp=10.,target_speed=3.,stop_required=False,emergency=False,active=True,decision_reason='follow')
        d.odom_stamp=d.status_stamp=10.
        d.odom_wall=d.status_wall=100.
        d.last_wall=99.95
        d.accel=d.brake=0.
        d.pub=Publisher();d.preview=Publisher();d.log=Log()

    def tick(self):
        with patch.object(self.module.time,'monotonic',return_value=100.):self.d.tick(None)
        return self.d.pub.messages[-1]

    def test_pedal_mode_and_rate_limit(self):
        cmd=self.tick()
        self.assertEqual(cmd.longlCmdType,1)
        self.assertGreater(cmd.accel,0)
        self.assertLessEqual(cmd.accel,.020001)
        self.assertEqual(cmd.brake,0)

    def test_emergency_bypasses_smoothing(self):
        self.d.status.update(stop_required=True,emergency=True,target_speed=0)
        cmd=self.tick()
        self.assertEqual((cmd.accel,cmd.brake),(0,1))

    def test_stale_status_brakes(self):
        self.d.status_stamp=8
        self.assertEqual(self.tick().brake,1)

    def test_dead_publisher_wall_watchdog(self):
        self.d.status_wall=90
        self.assertEqual(self.tick().brake,1)

    def test_zero_speed_normal_stop_holds(self):
        self.d.status.update(stop_required=True,target_speed=0)
        self.d.ego.speed=0
        cmd=self.tick()
        self.assertEqual((cmd.accel,cmd.brake),(0,CFG['hold_brake']))

    def test_sudden_slow_lead_produces_brake(self):
        self.d.status.update(target_speed=0)
        self.d.ego.speed=3
        cmd=self.tick()
        self.assertEqual(cmd.accel,0)
        self.assertGreater(cmd.brake,0)

    def test_dryrun_only_preview(self):
        self.d.pub=None
        with patch.object(self.module.time,'monotonic',return_value=100.):self.d.tick(None)
        self.assertEqual(len(self.d.preview.messages),1)

    def test_goal_stops(self):
        self.d.ego=Ego(100,0,0,0)
        self.assertEqual(self.tick().brake,CFG['hold_brake'])
