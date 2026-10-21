"""RViz shows the ego conflict traversal currently accepted by the decision.

Green is a confirmed permission, not a claim of observed free space behind an
occluder. Yellow is confirmation pending; red is blocked or emergency.
"""
def markers(engine, row, frame, stamp):
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    import rospy
    array=MarkerArray()
    m=Marker()
    m.header.frame_id=frame; m.header.stamp=stamp
    m.ns='entry_gap'; m.id=0; m.type=Marker.LINE_STRIP; m.action=Marker.ADD
    m.pose.orientation.w=1.; m.scale.x=0.55; m.color.a=0.9
    usable=(row['active'] and row['commit_latched'] and not row['stop_required'] and not row['emergency_active'])
    blocked=row['emergency_active'] or row['dangerous_vehicle_count']>0
    m.color.r=0. if usable else 1.
    m.color.g=1. if usable or not blocked else 0.
    m.color.b=0.
    lo=engine.m['stop_line']; hi=engine.m['conflict_end']
    for p,s in zip(engine.path.points,engine.path.s):
        if lo<=s<=hi:
            q=Point();q.x=p.x;q.y=p.y;q.z=getattr(p,'z',0.)+0.3;m.points.append(q)
    m.lifetime=rospy.Duration(0.5)
    array.markers.append(m)
    label=Marker();label.header=m.header;label.ns='entry_gap';label.id=1
    label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD
    label.pose.orientation.w=1.;label.scale.z=1.;label.color=m.color
    if m.points:
        label.pose.position.x=m.points[0].x;label.pose.position.y=m.points[0].y
    label.pose.position.z=(m.points[0].z+2.2) if m.points else 2.5;label.lifetime=m.lifetime
    label.text='{} | {} | hazards={} | safe={:.2f}s'.format(row['state'],row['decision_reason'],row['dangerous_vehicle_count'],row['safe_duration'])
    array.markers.append(label)
    if not row['active']:
        for item in array.markers:item.action=Marker.DELETE
    return array
