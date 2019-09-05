#!/usr/bin/env python
import rospy
import gtsam
import numpy as np
import tf
import tf_conversions
import tf_conversions.posemath as pm
#import ros_numpy
import time
import matplotlib.pyplot as plt
import gtsam.utils.plot as gtsam_plot

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

print("init graph")
initial = gtsam.Values()

currPose = PoseStamped()
hEdgeMsg = []
#hEdgeMsg = PoseWithCovarianceStamped()

time_old = time.time()
num = 0

def edge_callback(edge_data):
    print("got edge!!")


def h_edge_callback(h_edge_data):
    print("got history edge!!")
    global hEdgeMsg

    history_num = h_edge_data.header.frame_id
    print(history_num)
    if history_num == '1':
        hEdgeMsg = []
        hEdgeMsg.append(h_edge_data.header.stamp.nsecs)
        hEdgeMsg.append(h_edge_data.pose)
        print(hEdgeMsg[0])
        print(hEdgeMsg[1])




def pose_callback(data):

    currPose = data
'''
    global time_old, num, initial
    
    time_now = time.time()

    if time_now-time_old >= 1:
        num = num + 1
        ori = data.pose.orientation

        quaternion = (ori.x, ori.y, ori.z, ori.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        node_phi = euler[2]
        node_x = data.pose.position.x
        node_y = data.pose.position.y
        initial.insert(num, gtsam.Pose2(node_x, node_y, node_phi))
        print("\nInitial Estimate:\n{}".format(initial))

        data_old = data
        time_old = time_now
'''

'''
def build_graph():

	print("build_graph !!!")

	# Create noise models
	ODOMETRY_NOISE = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
	PRIOR_NOISE = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))

	# Create an empty nonlinear factor graph
	graph = gtsam.NonlinearFactorGraph()

	priorMean = gtsam.Pose2(0.0, 0.0, 0.0)  # prior at origin
	graph.add(gtsam.PriorFactorPose2(1, priorMean, PRIOR_NOISE))

	odometry = gtsam.Pose2(2.0, 0.0, 0.0)
	odometry1 = gtsam.Pose2(edge_list[0].position.x, edge_list[0].position.y, edge_list[0].position.z)
	odometry2 = gtsam.Pose2(edge_list[1].position.x, edge_list[1].position.y, edge_list[1].position.z)

	graph.add(gtsam.BetweenFactorPose2(1, 2, odometry1, ODOMETRY_NOISE))
	graph.add(gtsam.BetweenFactorPose2(2, 3, odometry2, ODOMETRY_NOISE))
	print("\nFactor Graph:\n{}".format(graph))

	# Create the data structure to hold the initialEstimate estimate to the solution
	# For illustrative purposes, these have been deliberately set to incorrect values
	initial = gtsam.Values()
	initial.insert(1, gtsam.Pose2(0.5, 0.0, 0.2))
	initial.insert(2, gtsam.Pose2(2.3, 0.1, -0.2))
	initial.insert(3, gtsam.Pose2(4.1, 0.1, 0.1))
	print("\nInitial Estimate:\n{}".format(initial))
'''


def graph_creater():

    global edge_msg

    rospy.init_node('PoseGraphOpt', anonymous=True)

    rospy.Subscriber("pose_stamped", PoseStamped, pose_callback)
    rospy.Subscriber("edge", PoseWithCovarianceStamped, edge_callback)
    rospy.Subscriber("history_edge", PoseWithCovarianceStamped, h_edge_callback)

    rospy.spin()

if __name__ == '__main__':
    graph_creater()

