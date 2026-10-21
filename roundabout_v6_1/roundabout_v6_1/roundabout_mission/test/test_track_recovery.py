"""Regression: track loss must not leave a permanent pre-entry brake latch."""
import copy
import math
import unittest
from pathlib import Path
import yaml
from roundabout_mission.core import Decision,Ego,Track,swept_boxes
from test_core import CFG,PATH,CIRC


class Recovery(unittest.TestCase):
    def setUp(self):
        self.c=copy.deepcopy(CFG)
        self.d=Decision(PATH,CIRC,self.c)
        self.e=Ego(15.9,0,0,0)

    def sample(self,now,tracks=(),fresh=True):
        self.d.update_tracks(tracks,now)
        return self.d.step(self.e,now,True,fresh)

    def lose(self):
        # A vehicle already about to occupy the conflict is critical history.
        self.sample(1.,[Track(101,32,-6,math.pi/2,4.5,1.8,0,3)])
        self.d.state='COMMIT'
        r=self.sample(2.)
        self.assertTrue(r['stop_required'])
        self.assertEqual(r['lost_track_ids'],[101])
        self.assertEqual(r['state'],'WAIT')
        return r

    def test_clear_scene_recovers_then_rechecks_before_commit(self):
        self.lose()
        recovered=False
        for n in range(1,33):
            r=self.sample(2.+n*.1)
            if r['recovered_track_ids']:
                recovered=True
                self.assertTrue(r['stop_required'])
                self.assertEqual(r['recovered_track_ids'],[101])
        self.assertTrue(recovered)
        self.assertEqual(r['state'],'COMMIT')
        self.assertFalse(r['stop_required'])
        self.assertEqual(r['lost_track_ids'],[])

    def test_timer_ticks_without_new_frames_cannot_release(self):
        self.lose()
        for n in range(1,100):
            r=self.d.step(self.e,2+n*.1)
        self.assertTrue(r['stop_required'])
        self.assertEqual(r['recovery_frames'],1)

    def test_duplicate_frames_cannot_release(self):
        self.lose()
        for n in range(100):
            self.d.update_tracks([],2.)
            r=self.d.step(self.e,2.+n*.01)
        self.assertEqual(r['recovery_frames'],1)
        self.assertTrue(r['stop_required'])

    def test_lidar_dropout_restarts_confirmation(self):
        self.lose()
        for n in range(1,16): self.sample(2+n*.1)
        r=self.d.step(self.e,3.6,True,False)
        self.assertTrue(r['stop_required'])
        r=self.sample(3.7)
        self.assertEqual(r['recovery_frames'],1)

    def test_different_vehicle_blocks_entry_but_not_scene_reconfirmation(self):
        self.lose()
        for n in range(1,60):
            r=self.sample(2+n*.1,[Track(202,32,-15,math.pi/2,4.5,1.8,0,3)])
        self.assertTrue(r['stop_required'])
        self.assertEqual(r['blocking_vehicle_id'],202)
        self.assertEqual(r['lost_track_ids'],[])
        self.assertEqual(r['decision_reason'],'waiting_for_safe_gap')
        self.assertFalse(r['emergency'])

    def test_stationary_obstacle_cannot_be_cleared_by_empty_lists(self):
        self.sample(1.,[Track(101,24,0,0,4.5,1.8,0,0)])
        for n in range(1,70): r=self.sample(1+n*.1)
        self.assertTrue(r['stop_required'])
        self.assertEqual(r['decision_reason'],'lost_stationary_obstacle_on_path')

    def test_inside_conflict_requires_reacquisition(self):
        self.e=Ego(31,0,0,2);self.d.state='COMMIT'
        self.sample(1.,[Track(101,44,0,0,4.5,1.8,1,0)])
        for n in range(1,70): r=self.sample(1+n*.1)
        self.assertTrue(r['stop_required'])
        self.assertEqual(r['decision_reason'],'track_loss_inside_conflict_reacquire')

    def test_same_id_returns_but_current_hazard_still_stops(self):
        self.lose()
        r=self.sample(2.1,[Track(101,32,-15,math.pi/2,4.5,1.8,0,3)])
        self.assertEqual(r['lost_track_ids'],[])
        self.assertTrue(r['stop_required'])

    def test_id_handoff_removes_old_history_and_evaluates_new_track(self):
        self.sample(1.,[Track(101,32,-20,math.pi/2,4.5,1.8,0,3)])
        r=self.sample(1.2,[Track(202,32,-19.4,math.pi/2,4.5,1.8,0,3)])
        self.assertEqual(r['id_handoffs'],[[101,202]])
        self.assertNotIn(101,self.d.cache)
        self.assertTrue(r['stop_required'])
        self.assertEqual(r['blocking_vehicle_id'],202)

    def test_ambiguous_ids_are_not_merged(self):
        self.sample(1.,[Track(101,32,-20,0,4.5,1.8,0,3),Track(102,32.5,-20,0,4.5,1.8,0,3)])
        r=self.sample(1.2,[Track(202,32.2,-19.4,0,4.5,1.8,0,3)])
        self.assertEqual(r['id_handoffs'],[])
        self.assertIn(101,self.d.cache)
        self.assertIn(102,self.d.cache)

    def test_one_old_id_cannot_transfer_to_two_new_ids(self):
        self.sample(1.,[Track(101,32,-20,0,4.5,1.8,0,3)])
        r=self.sample(1.2,[Track(202,32,-19.4,0,4.5,1.8,0,3),Track(203,32.1,-19.4,0,4.5,1.8,0,3)])
        self.assertEqual(r['id_handoffs'],[])

    def test_authorized_stop_after_entry_before_conflict_can_recheck(self):
        self.e=Ego(25,0,0,0)
        self.lose()
        for n in range(1,33): r=self.sample(2+n*.1)
        self.assertEqual(r['state'],'COMMIT')
        self.assertFalse(r['stop_required'])

    def test_thirty_one_old_moving_ids_do_not_latch_forever(self):
        self.d.lost={i:(Track(i,32,-20,math.pi/2,4.5,1.8,0,3),0.) for i in range(31)}
        recovered=[]
        for n in range(40):
            r=self.sample(2+n*.1)
            recovered.extend(r['recovered_track_ids'])
        self.assertEqual(len(recovered),31)
        self.assertFalse(r['stop_required'])


class UploadedSnapshot(unittest.TestCase):
    def test_snapshot_is_32ms_not_evidence_of_id_switching(self):
        frames=yaml.safe_load((Path(__file__).parent/'fixtures/tracks_20261003.yaml').read_text())
        ids=[{o['id'] for o in f['obstacles']} for f in frames]
        self.assertTrue(all(i==ids[0] for i in ids))
        for f in frames:
            car=next(o for o in f['obstacles'] if o['id']==534)
            self.assertAlmostEqual(math.hypot(car['velocity_x_map'],car['velocity_y_map']),5.24146,places=3)
            e=Ego(-99.17146,315.91374,math.pi/2,0.)
            for o in f['obstacles']:
                if o['id'] not in (18,20):continue
                t=Track(o['id'],o['center_x_map'],o['center_y_map'],o['yaw'],o['length'],o['width'],o['velocity_x_map'],o['velocity_y_map'])
                self.assertGreater(swept_boxes(e,t,CFG)[0],0.)


if __name__=='__main__':unittest.main()
