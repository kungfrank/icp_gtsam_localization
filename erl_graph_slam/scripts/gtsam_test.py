#!/usr/bin/env python
import rospy
import gtsam
import numpy as np
import tf
import tf_conversions
import tf_conversions.posemath as pm
import ros_numpy
import time
import matplotlib.pyplot as plt
import gtsam.utils.plot as gtsam_plot

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

#rate = rospy.Rate(rospy.get_param('~hz',10))

data_old = PoseStamped()
edge_msg = PoseStamped()
time_old = time.time()
num = 0
edge_list = []

def callback(data):

    global data_old, edge, time_old, num, edge_list
    
    time_now = time.time()

    if num == 0:
		num=num + 1
    if time_now-time_old >= 1 and num < 3 and num > 0:
        x = ros_numpy.geometry.pose_to_numpy(data.pose)
        x_old = ros_numpy.geometry.pose_to_numpy(data_old.pose)
        x_old_inv = np.linalg.inv(x_old)
        edge = np.dot(x_old_inv, x)
        print("ans:")
        print(edge)
        edge_msg.header = data_old.header
        edge_msg.pose = ros_numpy.geometry.numpy_to_pose(edge)

        edge_list.append(edge_msg.pose)

        data_old = data
        time_old = time_now
        num=num + 1
    if num == 3: # build graph
		build_graph()

def build_graph():

	print("build_graph !!!")
	# Create noise models
	ODOMETRY_NOISE = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
	PRIOR_NOISE = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))

	# Create an empty nonlinear factor graph
	graph = gtsam.NonlinearFactorGraph()

	# Add a prior on the first pose, setting it to the origin
	# A prior factor consists of a mean and a noise model (covariance matrix)
	priorMean = gtsam.Pose2(0.0, 0.0, 0.0)  # prior at origin
	graph.add(gtsam.PriorFactorPose2(1, priorMean, PRIOR_NOISE))

	# Add odometry factors
	odometry = gtsam.Pose2(2.0, 0.0, 0.0)
	odometry1 = gtsam.Pose2(edge_list[0].position.x, edge_list[0].position.y, edge_list[0].position.z)
	odometry2 = gtsam.Pose2(edge_list[1].position.x, edge_list[1].position.y, edge_list[1].position.z)

	# For simplicity, we will use the same noise model for each odometry factor
	# Create odometry (Between) factors between consecutive poses
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
	
	# optimize using Levenberg-Marquardt optimization
	params = gtsam.LevenbergMarquardtParams()
	optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
	result = optimizer.optimize()
	print("\nFinal Result:\n{}".format(result))
	
	# 5. Calculate and print marginal covariances for all variables
	marginals = gtsam.Marginals(graph, result)
	for i in range(1, 4):
	    print("X{} covariance:\n{}\n".format(i, marginals.marginalCovariance(i)))
	
	fig = plt.figure(0)
	for i in range(1, 4):
	    gtsam_plot.plot_pose2(0, result.atPose2(i), 0.5, marginals.marginalCovariance(i))
	plt.axis('equal')
	plt.show()


def graph_creater():

    global edge_msg

    rospy.init_node('edge_publisher', anonymous=True)
    rospy.Subscriber("pose_stamped", PoseStamped, callback)
    print("aa")
    rospy.spin()

if __name__ == '__main__':
    graph_creater()

