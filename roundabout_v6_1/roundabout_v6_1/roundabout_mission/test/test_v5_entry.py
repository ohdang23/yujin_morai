"""Standalone synthetic decision regressions; no ROS or external repo needed."""
import copy
import json
import math
import unittest
from pathlib import Path
from types import SimpleNamespace as P
import yaml
from roundabout_mission.core import Decision, Ego, Track

BASE=yaml.safe_load((Path(__file__).parents[1]/'config/roundabout.yaml').read_text())
BASE.update(mission_indices=dict(approach=5,stop_line=20,entry=25,conflict_start=30,conflict_end=35,merge_complete=45,exit=70),circulating_conflict_indices=[45,55],roi_map=[-10,90,-60,60])
PATH=[P(x=x,y=0) for x in range(101)]
CIRC=[P(x=32,y=y) for y in range(-50,51)]
def car(id=54,y=-20,vy=3,x=32,vx=0,length=4.5,width=1.8):
    return Track(id,x,y,math.atan2(vy,vx),length,width,vx,vy)

class EntryV5(unittest.TestCase):
    def setUp(self):
        self.d=Decision(PATH,CIRC,copy.deepcopy(BASE));self.ego=Ego(15.9,0,0,0)
    def step(self,t,tracks=()):
        self.d.update_tracks(tracks,t)
        r=self.d.step(self.ego,t)
        json.dumps(r,allow_nan=False)
        return r
    def commit(self):
        self.step(1)
        self.assertEqual(self.step(1.3)['state'],'COMMIT')
    def test_case1_approach_pass_clear_commit(self):
        self.assertGreater(self.step(1,[car()])['dangerous_vehicle_count'],0)
        self.assertFalse(self.step(2,[car(y=30)])['commit_latched'])
        self.assertTrue(self.step(2.3,[car(y=31)])['commit_latched'])
    def test_case2_id_change_remains_unsafe(self):
        self.step(1,[car()])
        r=self.step(1.2,[car(id=71,y=-19.4)])
        self.assertGreater(r['dangerous_vehicle_count'],0)
        self.assertEqual(r['safe_duration'],0)
    def test_case3_short_occlusion(self):
        self.step(1,[car()])
        for t in (1.1,1.3,1.5,1.7):
            self.assertGreater(self.step(t)['dangerous_vehicle_count'],0)
        self.assertFalse(self.step(1.75,[car(id=71,y=-17.75)])['commit_latched'])
    def test_case4_split_partial_boxes(self):
        self.step(1,[car()])
        r=self.step(1.2,[car(id=71,y=-19.5,length=.3,width=.2),car(id=72,y=-18.5,length=.2,width=.3)])
        self.assertGreater(r['dangerous_vehicle_count'],0)
        self.assertFalse(r['commit_latched'])
    def test_case5_exact_confirmation(self):
        self.step(1)
        self.assertFalse(self.step(1.29)['commit_latched'])
        self.assertTrue(self.step(1.3)['commit_latched'])
    def test_case6_latch_id_change(self):
        self.commit()
        for t,i in ((1.4,54),(1.5,71),(1.6,99)):
            r=self.step(t,[car(id=i)])
            self.assertEqual(r['state'],'COMMIT')
            self.assertTrue(r['commit_latched'])
    def test_case7_emergency_after_commit(self):
        self.commit();self.ego=Ego(23,0,0,2)
        r=self.step(1.4,[car(x=25,y=0,vy=0)])
        self.assertTrue(r['emergency_active']);self.assertTrue(r['stop_required'])
        self.assertTrue(r['commit_latched']);self.assertEqual(r['state'],'COMMIT')
        self.assertEqual(r['target_speed'],0)
    def test_case8_roadside_stationary_noise(self):
        for t in (1.,1.3):r=self.step(t,[car(x=32,y=-20,vy=.1)])
        self.assertEqual(r['conflict_candidate_count'],0)
        self.assertTrue(r['commit_latched'])
    def test_stationary_on_path_retained_after_loss(self):
        self.step(1,[car(x=32,y=0,vy=0)])
        for t in (2.,2.4,3.):
            r=self.step(t)
            self.assertFalse(r['commit_latched'])
            self.assertGreater(r['dangerous_vehicle_count'],0)
    def test_id_permutation_has_identical_outputs(self):
        other=Decision(PATH,CIRC,copy.deepcopy(BASE))
        for t,y in ((1.,-20),(1.2,-19.4),(1.4,-18.8),(2.5,30),(2.8,31)):
            a=self.step(t,[car(id=54,y=y)])
            other.update_tracks([car(id=int(t*100),y=y)],t)
            b=other.step(self.ego,t)
            for key in ('state','safe_duration','dangerous_vehicle_count','target_speed','commit_latched','emergency_active'):
                self.assertEqual(a[key],b[key],key)
    def test_safe_timer_not_reset_by_harmless_id_change(self):
        self.step(1,[car(id=54,y=30)])
        self.assertTrue(self.step(1.3,[car(id=71,y=31)])['commit_latched'])
    def test_timer_ticks_and_duplicates_do_not_confirm(self):
        self.step(1)
        self.assertFalse(self.d.step(self.ego,1.4)['commit_latched'])
        self.assertFalse(self.step(1)['commit_latched'])
    def test_watchdog_brakes_latch_preserved(self):
        self.commit()
        r=self.d.step(self.ego,1.4,True,False)
        self.assertTrue(r['emergency_active']);self.assertTrue(r['commit_latched'])
    def test_nearly_stopped_velocity_during_occlusion_keeps_hazard(self):
        self.step(1,[car()])
        self.assertGreater(self.step(1.3,[car(id=71,vy=.01)])['dangerous_vehicle_count'],0)
    def test_emergency_stop_command_fields(self):
        # The separate existing driver adapter test asserts accel=0, brake=1.
        self.commit();self.ego=Ego(23,0,0,2)
        r=self.step(1.4,[car(x=25,y=0,vy=0)])
        self.assertTrue(r['emergency']);self.assertEqual(r['target_speed'],0.)

if __name__=='__main__':unittest.main()
