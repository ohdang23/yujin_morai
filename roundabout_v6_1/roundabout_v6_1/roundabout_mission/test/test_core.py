import copy
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import yaml
from purepursuit_mgeo.path import PathPoint
from roundabout_mission.core import Decision,Ego,Track,eta
from roundabout_mission.ros_io import fresh,parse_tracks,parse_odom

CFG = yaml.safe_load((Path(__file__).parents[1]/'config/roundabout.yaml').read_text())
CFG.update(trajectory_prediction_enabled=False,mission_indices=dict(approach=5,stop_line=20,entry=25,conflict_start=30,conflict_end=35,merge_complete=45,exit=70),circulating_conflict_indices=[45,55],roi_map=[-10,90,-60,60])
PATH = [PathPoint(x,0,0) for x in range(101)]
CIRC = [PathPoint(32,y,0) for y in range(-50,51)]


def car(id=1,x=32,y=-20,vx=0,vy=4):
    return Track(id,x,y,math.atan2(vy,vx),4.5,1.8,vx,vy)


class Scenarios(unittest.TestCase):
    def setUp(self):
        self.d = Decision(PATH,CIRC,copy.deepcopy(CFG))
        self.e = Ego(15.9,0,0,0.)

    def step(self,tracks=(),now=1.,ego=None):
        self.d.update_tracks(tracks,now)
        return self.d.step(ego or self.e,now)

    def ready(self):
        self.step(now=1)
        return self.step(now=2)

    def test_case1_empty_commits(self):
        self.assertEqual(self.ready()['state'],'COMMIT')

    def test_case2_close_approach_waits_then_goes(self):
        t = car(y=-20,vy=3)
        r = self.step([t])
        self.assertTrue(r['stop_required'])
        self.assertEqual(r['blocking_vehicle_id'],1)
        self.step([car(y=20)],now=2)
        self.assertEqual(self.step([car(y=24)],now=3)['state'],'COMMIT')

    def test_case3_far_car_allows(self):
        self.step([car(y=-49,vy=1)])
        self.assertEqual(self.step([car(y=-48,vy=1)],now=2)['state'],'COMMIT')

    def test_case4_all_vehicles_checked(self):
        r = self.step([car(id=1,y=-49,vy=1),car(id=2,y=-20,vy=3)])
        self.assertTrue(r['stop_required'])
        self.assertEqual(r['blocking_vehicle_id'],2)

    def test_case5_slow_lead_limits_speed(self):
        self.d.state='FOLLOW';self.d.target=3
        r=self.step([car(x=56,y=0,vx=1,vy=0)],ego=Ego(45,0,0,3))
        self.assertEqual(r['lead_vehicle_id'],1)
        self.assertLess(r['target_speed'],3)

    def test_case6_lead_accelerates_smoothly(self):
        self.d.state='FOLLOW';self.d.target=1
        r=self.step([car(x=65,y=0,vx=4,vy=0)],ego=Ego(45,0,0,1))
        self.assertGreater(r['target_speed'],1)
        self.assertLessEqual(r['target_speed'],1+.8*.05+1e-8)

    def test_case7_lead_brakes(self):
        self.d.state='FOLLOW';self.d.target=3
        r=self.step([car(x=55,y=0,vx=0,vy=0)],ego=Ego(45,0,0,3))
        self.assertEqual(r['target_speed'],0)

    def test_case8_missing_lead_held(self):
        self.d.state='FOLLOW';self.d.target=2
        self.step([car(x=57,y=0,vx=1,vy=0)],ego=Ego(45,0,0,2))
        r=self.step(now=1.2,ego=Ego(45,0,0,2))
        self.assertEqual(r['lead_vehicle_id'],1)
        self.assertLessEqual(r['target_speed'],2)

    def test_case9_rear_detection_does_not_cancel_commit(self):
        self.ready()
        r=self.step([car(x=5,y=0,vx=1,vy=0)],now=2.1,ego=Ego(23,0,0,2))
        self.assertEqual(r['state'],'COMMIT')
        self.assertFalse(r['stop_required'])

    def test_case10_imminent_collision_stops_commit(self):
        self.ready()
        r=self.step([car(x=25,y=0,vx=0,vy=0)],now=2.1,ego=Ego(23,0,0,2))
        self.assertTrue(r['emergency'])

    def test_stale_input_during_commit(self):
        self.ready()
        self.assertTrue(self.d.step(self.e,2.1,False)['stop_required'])

    def test_track_hold_expiry_does_not_release(self):
        self.d.state='FOLLOW'
        e=Ego(45,0,0,2)
        self.step([car(x=57,y=0,vx=1,vy=0)],ego=e)
        r=self.step(now=2,ego=e)
        self.assertTrue(r['stop_required'])
        self.assertEqual('track_loss_inside_conflict_reacquire',r['decision_reason'])

    def test_passed_vehicle_excluded(self):
        self.assertIsNone(self.step([car(y=20)])['blocking_vehicle_id'])

    def test_wrong_direction_excluded(self):
        self.assertIsNone(self.step([car(y=-20,vy=-3)])['blocking_vehicle_id'])

    def test_static_conflict_blocks(self):
        self.assertTrue(self.step([car(y=0,vy=0)])['stop_required'])

    def test_static_off_path_not_candidate(self):
        self.assertIsNone(self.step([car(x=80,y=30,vy=0)])['blocking_vehicle_id'])

    def test_side_lidar_fragment_not_emergency(self):
        fragment = Track(99, 15.9, 2.2, 0.0, 0.25, 0.02, 0.0, 0.0)
        r = self.step([fragment])
        self.assertFalse(r['emergency'])
        self.assertIsNone(r['minimum_ttc'])

    def test_adjacent_car_not_lead(self):
        self.d.state='FOLLOW'
        r=self.step([car(x=65,y=6,vx=2,vy=0)],ego=Ego(45,0,0,2))
        self.assertIsNone(r['lead_vehicle_id'])

    def test_safe_debounce_reset(self):
        self.step()
        self.step([car(y=-20,vy=3)],now=1.7)
        self.assertNotEqual(self.step([car(y=25)],now=1.9)['state'],'COMMIT')

    def test_exit(self):
        self.d.state='FOLLOW'
        self.assertEqual(self.step(ego=Ego(75,0,0,2))['state'],'EXIT')

    def test_spawn_inside_not_authorized(self):
        self.assertTrue(self.step(ego=Ego(30,0,0,0))['stop_required'])

    def test_off_path(self):
        self.assertTrue(self.step(ego=Ego(10,20,0,0))['stop_required'])

    def test_eta_from_rest(self):
        self.assertAlmostEqual(eta(4,0,4,2),2.)
        self.assertAlmostEqual(eta(10,0,2,1),6.)

    def test_invalid_config_rejected(self):
        bad=copy.deepcopy(CFG);bad['mission_indices']['stop_line']=-1
        with self.assertRaises(ValueError): Decision(PATH,CIRC,bad)

    def test_header_freshness_and_wall_watchdog(self):
        self.assertTrue(fresh(1,10,1.1,10.1,.5))
        self.assertFalse(fresh(1,10,1.1,11,.5))
        self.assertFalse(fresh(2,10,1,10,.5))

    def test_parse_actual_fields(self):
        o=NS(id=1,center_x_map=1,center_y_map=2,yaw=0,length=4,width=2,velocity_x_map=3,velocity_y_map=0)
        msg=NS(header=NS(frame_id='map'),obstacle_count=1,obstacles=[o])
        self.assertEqual(parse_tracks(msg,'map')[0].vx,3)
        msg.header.frame_id='lidar'
        with self.assertRaises(ValueError):parse_tracks(msg,'map')

    def test_nan_rejected(self):
        o=NS(id=1,center_x_map=float('nan'),center_y_map=2,yaw=0,length=4,width=2,velocity_x_map=3,velocity_y_map=0)
        with self.assertRaises(ValueError):parse_tracks(NS(header=NS(frame_id='map'),obstacle_count=1,obstacles=[o]),'map')


if __name__ == '__main__':unittest.main()
