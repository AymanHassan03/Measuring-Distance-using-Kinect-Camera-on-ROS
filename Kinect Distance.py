#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64

rospy.init_node('kinect_distance')
rate = rospy.Rate(500)

def callback(data,Distance):
    p = ros_numpy.numpify(data)
    height = p.shape[0]
    width = p.shape[1]
    points = np.zeros((height*width, 3), dtype=np.float32)
    points [: , 0] = np.resize(p['x'], height*width)
    points [: , 1] = np.resize(p['y'], height*width)
    points [: , 2] = np.resize(p['z'], height*width)
    x = points[0][0]
    y = points[0][1]
    z = points[0][2]
    Distance = (np.sqrt(pow(x,2)+pow(y,2)+pow(z,2)))*100

    pub = rospy.Publisher('Distance', Int64, queue_size=1)
    pub.publish(Distance)

rospy.Subscriber("/camera/depth/points", PointCloud2, callback)
rospy.spin()
