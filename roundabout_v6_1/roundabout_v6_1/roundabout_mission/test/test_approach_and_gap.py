"""Regression for approach stop/go and loss confirmation missing every gap."""
import copy
import math
import unittest
from roundabout_mission.core import Decision,Ego,Track
from test_core import CFG,PATH,CIRC
from traffic_benchmark import fixture,driver_module


class ApproachAndGap(unittest.TestCase):
    def setUp(self):
        self.c=copy.deepcopy(CFG)
        self.d=Decision(PATH,CIRC,self.c)

    def frame(self,e,now,tracks=()):
        self.d.update_tracks(tracks,now)
        return self.d.step(e,now)

    def test_blocked_and_clear_frames_do_not_toggle_approach_to_merge(self):
        e=Ego(10,0,0,2)
        self.d.target=2.
        for n in range(70):
            # Alternating empty/blocked windows longer than safe_confirm_s.
            tracks=[] if (n//12)%2 else [Track(1,32,-6,math.pi/2,4.5,1.8,0,3)]
            r=self.frame(e,1+n*.1,tracks)
            self.assertEqual(r['state'],'APPROACH')
            self.assertFalse(r['gap_check_active'])
            self.assertFalse(r['stop_required'])
            self.assertFalse(r['emergency'])
            self.assertEqual(r['target_speed'],2.)

    def test_lost_moving_history_does_not_full_brake_approach(self):
        self.d.lost={1430:(Track(1430,32,-6,math.pi/2,4.5,1.8,0,3),0.)}
        self.d.target=2.
        e=Ego(10,0,0,2)
        r=self.frame(e,1.)
        self.assertEqual(r['state'],'APPROACH')
        self.assertEqual(r['target_speed'],2.)
        self.assertFalse(r['stop_required'])
        self.assertFalse(r['emergency'])

    def test_loss_reconfirmation_progresses_during_blocked_traffic(self):
        e=Ego(15.9,0,0,0)
        self.d.lost={1430:(Track(1430,32,-6,math.pi/2,4.5,1.8,0,3),0.)}
        for n in range(25):
            r=self.frame(e,1+n*.1,[Track(585,32,-6,math.pi/2,4.5,1.8,0,3)])
            self.assertTrue(r['stop_required'])
            self.assertFalse(r['emergency'])
        self.assertEqual(r['lost_track_ids'],[])
        self.assertEqual(r['blocking_vehicle_id'],585)
        self.assertEqual(r['decision_reason'],'waiting_for_safe_gap')
        # The next gap needs only the normal confirmation, not 2s + 0.8s.
        for n in range(10):
            r=self.frame(e,3.5+n*.1,[Track(585,32,20+4*n*.1,math.pi/2,4.5,1.8,0,4)])
        self.assertEqual(r['state'],'COMMIT')
        self.assertFalse(r['stop_required'])
        self.assertEqual(r['target_speed'],self.c['merge_speed'])

    def test_safe_confirmation_requires_new_sensor_frames(self):
        e=Ego(15.9,0,0,0)
        self.frame(e,1.)
        for n in range(30):r=self.d.step(e,1+n*.1)
        self.assertTrue(r['stop_required'])
        self.assertEqual(r['safe_frames'],1)

    def test_stationary_obstacle_and_stale_inputs_still_stop(self):
        e=Ego(10,0,0,2)
        r=self.frame(e,1.,[Track(9,13,0,0,1.,1.,0,0)])
        self.assertTrue(r['emergency'])
        self.assertTrue(r['stop_required'])
        r=self.d.step(e,2.,True,False)
        self.assertEqual(r['decision_reason'],'obstacles_stale_or_invalid')
        self.assertTrue(r['emergency'])

    def test_log_position_confirms_loss_without_claiming_imminent_collision(self):
        path,circ,c=fixture()
        c['maximum_merge_speed']=3.
        d=Decision(path,circ,c)
        e=Ego(-99.36302821836726,307.7783703435275,math.pi/2,.00273146116)
        d.lost={i:(Track(i,-90,330,0,4.5,1.8,3,0),0.) for i in (1430,1451)}
        x,y,h=d.predictor.circ_path.sample(40.)
        tr=Track(585,float(x),float(y),float(h),4.5,1.8,4*math.cos(h),4*math.sin(h))
        d.update_tracks([tr],1.)
        r=d.step(e,1.)
        self.assertAlmostEqual(r['stop_line_distance_m'],.326870887,places=5)
        self.assertTrue(r['stop_required'])
        self.assertFalse(r['emergency'])
        self.assertEqual(r['track_recovery_reason'],'track_loss_scene_confirming')
        self.assertEqual(r['blocking_track_details'][0]['id'],585)

    def test_accepted_gap_requests_acceleration_through_actual_driver(self):
        path,circ,c=fixture()
        d=Decision(path,circ,c)
        p=path[1778]
        e=Ego(p.x,p.y,math.pi/2,0.)
        for n in range(10):
            now=1+n*.1
            d.update_tracks([],now)
            r=d.step(e,now)
        self.assertEqual(r['state'],'COMMIT')
        self.assertEqual(r['target_speed'],c['maximum_merge_speed'])
        clock=[now];module=driver_module(c,clock);driver=module.Driver()
        driver.ego=e;driver.odom_stamp=now;driver.odom_wall=now
        driver.status=r;driver.status_stamp=now;driver.status_wall=now
        clock[0]+=.05
        driver.tick(None)
        cmd=driver.preview.last
        self.assertGreater(cmd.accel,0.)
        self.assertEqual(cmd.brake,0.)
        self.assertEqual(cmd.longlCmdType,1)

    def test_fast_candidate_can_clear_fast_car_without_relaxing_boxes(self):
        path,circ,c=fixture()
        d=Decision(path,circ,c)
        s=d.m['stop_line']-c['ego_front_m']-c['stop_margin_m']
        x,y,h=d.predictor.ego_path.sample(s)
        e=Ego(float(x),float(y),float(h),0.)
        x,y,h=d.predictor.circ_path.sample(0.)
        tr=Track(585,float(x),float(y),float(h),5.67,1.779,6.237*math.cos(h),6.237*math.sin(h))
        # Identical footprint/uncertainty model at both speeds.
        self.assertIn(585,d.predictor.assess(e,[tr],s,d.m['exit'],3.)['predicted_blocking_ids'])
        self.assertEqual(d.predictor.assess(e,[tr],s,d.m['exit'],4.)['predicted_blocking_ids'],[])
        for n in range(10):
            d.update_tracks([tr],1+n*.1)
            r=d.step(e,1+n*.1)
        self.assertFalse(r['stop_required'])
        self.assertEqual(r['state'],'COMMIT')
        self.assertGreater(r['target_speed'],3.)
        self.assertLessEqual(r['target_speed'],c['test_speed_limit'])

    def test_explicit_three_mps_ceiling_is_respected(self):
        path,circ,c=fixture();c['test_speed_limit']=3.
        d=Decision(path,circ,c)
        s=d.m['stop_line']-c['ego_front_m']-c['stop_margin_m']
        x,y,h=d.predictor.ego_path.sample(s);e=Ego(float(x),float(y),float(h),0.)
        x,y,h=d.predictor.circ_path.sample(0.)
        tr=Track(585,float(x),float(y),float(h),5.67,1.779,6.237*math.cos(h),6.237*math.sin(h))
        for n in range(10):
            d.update_tracks([tr],1+n*.1);r=d.step(e,1+n*.1)
        self.assertTrue(r['stop_required'])
        self.assertEqual(r['selected_merge_speed_mps'],3.)

    def test_fast_candidate_is_rejected_when_it_catches_a_slow_lead(self):
        path,circ,c=fixture();d=Decision(path,circ,c)
        s=d.m['stop_line']-c['ego_front_m']-c['stop_margin_m']
        x,y,h=d.predictor.ego_path.sample(s);e=Ego(float(x),float(y),float(h),0.)
        x,y,h=d.predictor.circ_path.sample(20.)
        tr=Track(999,float(x),float(y),float(h),4.5,1.8,2*math.cos(h),2*math.sin(h))
        self.assertIn(999,d.predictor.assess(e,[tr],s,d.m['exit'],4.)['predicted_blocking_ids'])
        self.assertEqual(d.predictor.assess(e,[tr],s,d.m['exit'],3.)['predicted_blocking_ids'],[])
        for n in range(10):
            d.update_tracks([tr],1+n*.1);r=d.step(e,1+n*.1)
        self.assertFalse(r['stop_required'])
        self.assertLess(r['selected_merge_speed_mps'],4.)
        self.assertEqual(r['predicted_blocking_ids'],[])

    def test_roadside_fragment_is_still_checked_as_geometry(self):
        self.d.lost={9:(Track(9,14.,2.2,0.,.25,.02,0.,0.),0.)}
        r=self.frame(Ego(10.,0.,0.,2.),1.)
        self.assertFalse(r['emergency'])
        self.assertFalse(r['stop_required'])


if __name__=='__main__':unittest.main()
