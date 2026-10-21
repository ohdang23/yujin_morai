import copy
import math
import unittest
from roundabout_mission.core import Decision,Ego,Track
from test_core import CFG,PATH,CIRC
from traffic_benchmark import fixture


class TrajectoryPrediction(unittest.TestCase):
    def setUp(self):
        self.path,self.circ,self.cfg=fixture()
        self.cfg['trajectory_prediction_enabled']=True
        self.d=Decision(self.path,self.circ,self.cfg)
        p=self.path[1778]
        self.e=Ego(p.x,p.y,math.pi/2,0.)
        self.s=self.d.path.project(p.x,p.y)[1]

    def car(self,s,speed=6.2,id=1):
        x,y,h=self.d.predictor.circ_path.sample(s)
        return Track(id,float(x),float(y),float(h),5.7,1.8,speed*math.cos(h),speed*math.sin(h))

    def assess(self,tracks):
        return self.d.predictor.assess(self.e,tracks,self.s,self.d.m['exit'],4.)

    def test_empty_road_has_complete_candidate(self):
        r=self.assess([])
        self.assertTrue(r['prediction_complete'])
        self.assertEqual(r['predicted_blocking_ids'],[])

    def test_safe_phase_allows_and_following_car_blocks(self):
        self.assertEqual(self.assess([self.car(0)])['predicted_blocking_ids'],[])
        r=self.assess([self.car(0),self.car(50,id=2)])
        self.assertIn(2,r['predicted_blocking_ids'])

    def test_car_past_old_conflict_is_not_ignored_for_next_lap(self):
        t=self.car(80)
        self.assertGreater(80,self.d.ce+t.length/2)
        r=self.assess([t])
        self.assertIn(t.id,r['predicted_blocking_ids'])

    def test_curved_missing_track_is_not_extrapolated_straight(self):
        t=self.car(25,speed=7.)
        x,y,h,vx,vy=self.d.predictor.propagate(t,.7)
        ex,ey,eh=self.d.predictor.circ_path.sample(29.9)
        self.assertAlmostEqual(x,float(ex),places=5)
        self.assertAlmostEqual(y,float(ey),places=5)
        self.assertAlmostEqual(math.hypot(vx,vy),7.)

    def test_curved_id_handoff_compares_propagated_velocity(self):
        t=self.car(25,speed=7.)
        self.d.update_tracks([t],1.)
        x,y,h,vx,vy=self.d.predictor.propagate(t,1.2)
        new=Track(202,x,y,h,t.length,t.width,vx,vy)
        self.assertGreater(math.hypot(new.vx-t.vx,new.vy-t.vy),self.cfg['reid_velocity_delta_mps'])
        self.d.update_tracks([new],2.2)
        self.assertEqual(self.d.id_handoffs,[[t.id,new.id]])

    def test_new_conflict_revokes_commit_before_physical_entry(self):
        self.d.state='COMMIT'
        self.d.update_tracks([self.car(50)],1.)
        r=self.d.step(self.e,1.)
        self.assertTrue(r['stop_required'])
        self.assertEqual(r['decision_reason'],'new_conflict_before_entry')
        self.assertEqual(r['state'],'WAIT')

    def test_incomplete_horizon_never_authorizes_entry(self):
        self.cfg['trajectory_horizon_s']=5.
        d=Decision(self.path,self.circ,self.cfg)
        p=self.path[1780]
        self.e=Ego(p.x,p.y,math.pi/2,0.)
        for n in range(30):
            d.update_tracks([],1+n*.1)
            r=d.step(self.e,1+n*.1)
        self.assertTrue(r['stop_required'])
        self.assertEqual(r['decision_reason'],'prediction_horizon_insufficient')

    def test_stationary_in_path_is_a_predicted_hazard(self):
        t=Track(4,self.path[1800].x,self.path[1800].y,math.pi/2,4.5,1.8,0,0)
        self.assertIn(4,self.assess([t])['predicted_blocking_ids'])

    def test_temporal_padding_catches_fast_crossing_between_samples(self):
        c=copy.deepcopy(CFG);c['trajectory_prediction_enabled']=True
        d=Decision(PATH,CIRC,c)
        # Crosses ego footprint at t=.05, between .2 s predictor samples.
        t=Track(4,1.2,2.,math.pi/2,.2,.2,0,-40.)
        r=d.predictor.assess(Ego(0,0,0,0),[t],0.,30.,3.)
        self.assertIn(4,r['predicted_blocking_ids'])

    def test_stationary_velocity_residual_does_not_drift_a_curb_into_route(self):
        c=copy.deepcopy(CFG);d=Decision(PATH,CIRC,c)
        t=Track(7,28.,2.3,0.,.5,.2,0.,-.07)
        r=d.predictor.assess(Ego(15.9,0.,0.,0.),[t],15.9,70.,3.)
        self.assertEqual(r['predicted_blocking_ids'],[])
        # A stationary box in the path still blocks; no static-object deletion.
        t=Track(7,28.,0.,0.,.5,.2,0.,-.07)
        r=d.predictor.assess(Ego(15.9,0.,0.,0.),[t],15.9,70.,3.)
        self.assertEqual(r['predicted_blocking_ids'],[7])


if __name__=='__main__':unittest.main()
