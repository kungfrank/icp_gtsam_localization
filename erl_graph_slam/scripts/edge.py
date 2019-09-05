#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf_conversions
import tf_conversions.posemath as pm
import ros_numpy
import time

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

#rate = rospy.Rate(rospy.get_param('~hz',10))

data_old = PoseStamped()
edge_msg = PoseStamped()
time_old = time.time()

def callback(data):

    global data_old, edge, time_old

    time_now = time.time()

    if time_now-time_old >= 1:

        x = ros_numpy.geometry.pose_to_numpy(data.pose)
        x_old = ros_numpy.geometry.pose_to_numpy(data_old.pose)

        x_old_inv = np.linalg.inv(x_old)
        edge = np.dot(x_old_inv, x)

        print("ans:")
        print(edge)

        edge_msg.header = data_old.header
        edge_msg.pose = ros_numpy.geometry.numpy_to_pose(edge)
        data_old = data
	time_old = time_now

def edge_publisher():

    global edge_msg

    rospy.init_node('edge_publisher', anonymous=True)
    rospy.Subscriber("pose_stamped", PoseStamped, callback)
    pub = rospy.Publisher('edge', PoseStamped, queue_size=10)
    #rospy.spin()

    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
	rospy.Subscriber("pose_stamped", PoseStamped, callback)
	
        pub.publish(edge_msg)
        r.sleep()


if __name__ == '__main__':
    edge_publisher()

