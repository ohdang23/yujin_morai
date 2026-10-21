#!/usr/bin/env python3
"""Read-only runtime report. No motion commands and no node termination."""
import importlib
import argparse
from dataclasses import asdict
import json
import os
import socket
import sys
import threading
import time
import rosgraph
import rospkg
import rospy
from std_msgs.msg import String
from morai_msgs.msg import CtrlCmd
from lidar_perception.msg import LidarObstacleArray
from roundabout_mission.ros_io import parse_tracks
from roundabout_mission import VERSION


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-seconds',type=float,default=0.)
    parser.add_argument('--output',default='/tmp/roundabout_diagnostic_v42.jsonl')
    args=parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if not 0 <= args.record_seconds <= 60:
        parser.error('--record-seconds must be 0..60')
    rospy.init_node('roundabout_diagnose',anonymous=True,disable_signals=True)
    socket.setdefaulttimeout(4)
    print('PACKAGE_VERSION:',VERSION)
    print('PACKAGE_PATH:',rospkg.RosPack().get_path('roundabout_mission'))
    for name in ('roundabout_mission.core','purepursuit_mgeo.path','path_planning.longitudinal_controller'):
        m=importlib.import_module(name)
        print('SOURCE:',name,os.path.realpath(m.__file__))
    for key in ('/roundabout_test_driver/enable_control','/roundabout_test_driver/command_topic',
                '/roundabout_test_driver/path_file','/morai_udp_drive_bridge/command_topic',
                '/morai_udp_drive_bridge/control_remote_ip','/morai_udp_drive_bridge/control_remote_port',
                '/roundabout_merge_gate/approach_speed','/roundabout_merge_gate/merge_speed',
                '/roundabout_merge_gate/maximum_merge_speed',
                '/roundabout_test_driver/test_speed_limit','/roundabout_merge_gate/lost_recheck_s'):
        print('PARAM:',key,rospy.get_param(key,'MISSING'))
    pubs,subs,_=rosgraph.Master(rospy.get_name()).getSystemState()
    topic=rospy.get_param('/roundabout_test_driver/command_topic','/ctrl_cmd')
    print('PUBLISHERS:',dict(pubs).get(topic,[]))
    print('SUBSCRIBERS:',dict(subs).get(topic,[]))
    for t,cls in [('/roundabout_merge_gate/status',String),('/roundabout_test_driver/status',String),(topic,CtrlCmd)]:
        try:
            msg=rospy.wait_for_message(t,cls,timeout=4.)
            print(t, json.dumps(json.loads(msg.data),ensure_ascii=False) if cls is String else str(msg))
        except rospy.ROSException as exc:
            print(t,'NO_MESSAGE',str(exc))
    if args.record_seconds:
        record(args,topic)


def record(args,command_topic):
    """Read only: correlate live decisions/commands with actual tracked boxes."""
    frame=rospy.get_param('/roundabout_merge_gate/map_frame','map')
    obstacle_topic=rospy.get_param('/roundabout_merge_gate/obstacle_topic','/perception/lidar/tracked_obstacles_map')
    lock=threading.Lock()
    counts={}
    subscriptions=[]
    # Exclusive creation prevents overwriting a previous diagnostic record.
    with open(args.output,'x',encoding='utf-8') as stream:
        def receive(msg,topic):
            try:
                if isinstance(msg,String):
                    data=json.loads(msg.data)
                elif isinstance(msg,CtrlCmd):
                    data={key:getattr(msg,key) for key in ('longlCmdType','accel','brake','steering','velocity','acceleration')}
                else:
                    data=dict(timestamp=msg.header.stamp.to_sec(),frame=msg.header.frame_id,
                              obstacles=[asdict(t) for t in parse_tracks(msg,frame)])
                row=dict(received_s=time.time(),topic=topic,data=data)
                with lock:
                    if not stream.closed:
                        stream.write(json.dumps(row,allow_nan=False)+'\n')
                        counts[topic]=counts.get(topic,0)+1
            except (ValueError,TypeError):
                rospy.logwarn_throttle(2.,'Invalid diagnostic input on '+topic)
        for topic,cls in [('/roundabout_merge_gate/status',String),('/roundabout_test_driver/status',String),
                          (command_topic,CtrlCmd),(obstacle_topic,LidarObstacleArray)]:
            subscriptions.append(rospy.Subscriber(topic,cls,receive,callback_args=topic,queue_size=10))
        print('RECORDING:',args.output,'seconds:',args.record_seconds)
        end=time.monotonic()+args.record_seconds
        try:
            while not rospy.is_shutdown() and time.monotonic()<end:
                time.sleep(.1)
        finally:
            for subscription in subscriptions:subscription.unregister()
            with lock:
                stream.flush()
                stream.close()
    print('RECORDED:',args.output,'messages:',counts)


if __name__=='__main__':
    main()
