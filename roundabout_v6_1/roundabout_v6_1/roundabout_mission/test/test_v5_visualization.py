import sys
import types
import unittest
from unittest.mock import patch
from roundabout_mission.visualization import markers
import test_v5_entry

class Marker:
    LINE_STRIP=4;TEXT_VIEW_FACING=9;ADD=0;DELETE=2
    def __init__(self):
        self.header=types.SimpleNamespace()
        self.pose=types.SimpleNamespace(orientation=types.SimpleNamespace(),position=types.SimpleNamespace())
        self.scale=types.SimpleNamespace();self.color=types.SimpleNamespace();self.points=[]

class Visualization(unittest.TestCase):
    def render(self,d,row):
        mods={k:types.ModuleType(k) for k in ('visualization_msgs','visualization_msgs.msg','geometry_msgs','geometry_msgs.msg','rospy')}
        mods['visualization_msgs.msg'].Marker=Marker
        mods['visualization_msgs.msg'].MarkerArray=lambda:types.SimpleNamespace(markers=[])
        mods['geometry_msgs.msg'].Point=types.SimpleNamespace
        mods['rospy'].Duration=lambda x:x
        with patch.dict(sys.modules,mods):return markers(d,row,'map',1)
    def test_confirmed_gap_green_emergency_red_and_expiry(self):
        t=test_v5_entry.EntryV5();t.setUp();t.step(1);r=t.step(1.3)
        m=self.render(t.d,r).markers[0]
        self.assertEqual((m.color.r,m.color.g),(0.,1.));self.assertGreater(len(m.points),1)
        self.assertEqual(m.lifetime,.5)
        r.update(emergency_active=True,stop_required=True)
        m=self.render(t.d,r).markers[0]
        self.assertEqual((m.color.r,m.color.g),(1.,0.))
        r['active']=False
        self.assertTrue(all(m.action==Marker.DELETE for m in self.render(t.d,r).markers))
