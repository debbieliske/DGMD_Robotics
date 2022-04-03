#!/usr/bin/env python
    
import rospy

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

pub = rospy.Publisher('/range', Range, queue_size=10)

def callback(data):
    pass
#    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.ranges[0])
#    range_msg = Range()
#    range_msg.header.stamp = rospy.Time.now()
#    range_msg.header.frame_id = 'laser_link'
#    range_msg.field_of_view = 0.0698132
#    range_msg.min_range=0.1
#    range_msg.range: Any 

rospy.init_node("sonar_node", anonymous=True)
rospy.Subscriber('/scan', LaserScan, callback)

rospy.spin()
