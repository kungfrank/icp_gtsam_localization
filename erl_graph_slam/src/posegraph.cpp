#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf/transform_datatypes.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Key.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include "std_msgs/String.h"

#include <math.h> 

using namespace std;
using namespace gtsam;

geometry_msgs::PoseWithCovarianceStamped::Ptr Edge[20];
geometry_msgs::PoseStamped::Ptr currPose, oldCurrPose;
geometry_msgs::PoseStamped::Ptr kfPose;

tf::Transform OptKfPoseTf;

int historyEdgeTime[20];
int nodeNum = 1;
double travel_dist_linear = 0;
double travel_dist_angular = 0;
int history;
int opt_flag = 0;
int pose_callback_flag = 0;

//set init graph
Values initial;
NonlinearFactorGraph graph;

ros::Publisher  opt_pose_pub;
ros::Publisher  opt_graph_pub;

void insertEdge(int nodeNum, int i, geometry_msgs::PoseWithCovarianceStamped::Ptr Edge){
	tf::Quaternion q(Edge->pose.pose.orientation.x,
					 Edge->pose.pose.orientation.y,
					 Edge->pose.pose.orientation.z,
					 Edge->pose.pose.orientation.w);
	tf::Matrix3x3 mat(q);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	Pose2 csmOdometry(Edge->pose.pose.position.x, Edge->pose.pose.position.y, yaw);
	noiseModel::Diagonal::shared_ptr csmOdometryNoise = noiseModel::Diagonal::Sigmas(Vector3(Edge->pose.covariance[0],
																							 Edge->pose.covariance[7],
																							 Edge->pose.covariance[35]));
	graph.emplace_shared<BetweenFactor<Pose2> >((nodeNum-i-1), nodeNum, csmOdometry, csmOdometryNoise);
	//graph.print("\n Edge Graph:\n");
}

double getYawFromMsg(geometry_msgs::PoseStamped::Ptr currPose){
	tf::Quaternion q(currPose->pose.orientation.x,
					 currPose->pose.orientation.y,
					 currPose->pose.orientation.z,
					 currPose->pose.orientation.w);
	tf::Matrix3x3 mat(q);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	return yaw;
}


void createTfFromXYTheta(
  double x, double y, double theta, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

void edgeCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg)
{
	// cout << "history: " << history << endl; 
	nodeNum++;
	historyEdgeTime[0] = msg->header.stamp.nsec;
	Edge[0] = msg;
	/*
    cout << "currPoseTime: " << currPose->header.stamp.nsec << endl;
	for(int i=0;i<=history;i++){
  		cout << i << " : " << "historyEdgeTime: " << historyEdgeTime[i] << endl;
	}
	*/

	if((historyEdgeTime[0]!=currPose->header.stamp.nsec)){ROS_WARN("Not SYNC !!!");}

	//------------------------------ insert init pose of node to graph ------------------------------//
	tf::Quaternion q(currPose->pose.orientation.x,
					 currPose->pose.orientation.y,
					 currPose->pose.orientation.z,
					 currPose->pose.orientation.w);
	tf::Matrix3x3 mat(q);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	initial.insert(nodeNum, Pose2(currPose->pose.position.x, currPose->pose.position.y, yaw));
	//initial.insert(nodeNum, Pose2(0.0, 0.0, 0.0));
	//initial.print("\nInitial Estimate:\n");
	
	//------------------------------ insert edges of nodes to graph -----------------------------------//
	if(nodeNum < history+2){
		for(int i=0;i<=(nodeNum-2);i++){
			if(historyEdgeTime[0] == historyEdgeTime[i]){insertEdge(nodeNum, i, Edge[i]);}
			else{ROS_WARN("Drop !!!"); cout << "drop edge between:" << nodeNum << " and " << (nodeNum-1-i) << endl;}
		}
		//graph.print("\n Edge Graph:\n");
	}
	else{
		for(int i=0;i<=history;i++){
			if(historyEdgeTime[0] == historyEdgeTime[i]){insertEdge(nodeNum, i, Edge[i]);}
			else{ROS_WARN("Drop !!!"); cout << "drop edge between:" << nodeNum << " and " << (nodeNum-1-i) << endl;}
		}
		//graph.print("\n Edge Graph:\n");
	}

	////////////////////////////////////////////////////////////////////////////////////////////
/*	geometry_msgs::PoseArray opt_graph_msg;
    opt_graph_msg.header.stamp    = currPose->header.stamp;
    opt_graph_msg.header.frame_id = currPose->header.frame_id;

	for(int i=1 ; i<=nodeNum ; i++){
		double x = graph.at<Pose2>(i).x();
		double y = graph.at<Pose2>(i).y();
		double theta = graph.at<Pose2>(i).theta();
		tf::Transform PoseTf;
		createTfFromXYTheta(x, y, theta, PoseTf);
		geometry_msgs::Pose pose;
		tf::poseTFToMsg(PoseTf, pose);
		opt_graph_msg.poses.push_back(pose);
	}

	opt_graph_pub.publish(opt_graph_msg);*/
	////////////////////////////////////////////////////////////////////////////////////////////

	//------------------------------ calculate travel dist ------------------------------//
	if(nodeNum==2){oldCurrPose = currPose;}
	//// angular part ////
	// Get newYaw
	tf::Quaternion nq(currPose->pose.orientation.x,
					 currPose->pose.orientation.y,
					 currPose->pose.orientation.z,
					 currPose->pose.orientation.w);
	tf::Matrix3x3 matNew(nq);
	double newRoll, newPitch, newYaw;
	matNew.getRPY(newRoll, newPitch, newYaw);
	// Get oldYaw
	tf::Quaternion oq(oldCurrPose->pose.orientation.x,
					 oldCurrPose->pose.orientation.y,
					 oldCurrPose->pose.orientation.z,
					 oldCurrPose->pose.orientation.w);
	tf::Matrix3x3 matOld(oq);
	double oldRoll, oldPitch, oldYaw;
	matOld.getRPY(oldRoll, oldPitch, oldYaw);
	double yawChange = (newYaw - oldYaw);
	travel_dist_angular = travel_dist_angular + yawChange;
	//// linear part ////
	double x_change = currPose->pose.position.x - oldCurrPose->pose.position.x;
	double y_change = currPose->pose.position.y - oldCurrPose->pose.position.y;
	double distChange = sqrt( x_change*x_change + y_change*y_change );
	travel_dist_linear = travel_dist_linear + distChange;

	oldCurrPose = currPose; // UPDATE oldCurrPose
	/*
	cout << "Yaw change: " << (newYaw - oldYaw) << endl;
	cout << "Dist change: " << sqrt( x_change*x_change + y_change*y_change ) << endl;
	cout << "travel_dist_angular: " << travel_dist_angular << endl;
	cout << "travel_dist_linear: " << travel_dist_linear << endl;
	*/
	//------------------------------ Optimization part ------------------------------//
	if(travel_dist_linear >= 0.1 || travel_dist_angular >= 0.175){ // TODO : should set by rosparam 0.7 0.4
		ROS_INFO("Optimization !!!!!!!!");

		//Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
		
		
		GaussNewtonParams parameters;
		parameters.relativeErrorTol = 1;
		parameters.maxIterations = 100;
		GaussNewtonOptimizer optimizer(graph, initial, parameters);
		Values result = optimizer.optimize();
		

		//result.print("Final Result:\n");

		//cout << "pose dim: " << result.at(nodeNum).dim() << endl;

		double x_kf = result.at<Pose2>(nodeNum).x();
		double y_kf = result.at<Pose2>(nodeNum).y();
		double theta_kf = result.at<Pose2>(nodeNum).theta();
      	createTfFromXYTheta(x_kf, y_kf, theta_kf, OptKfPoseTf);

 	   	cout << "OPT KF: " << x_kf << ", " << y_kf << ", " << theta_kf << endl;

        //std::ostream& os = std::cout;
		//graph.saveGraph(os,result);

		////////////////////////////////////////////////////////////////////////////////////////////
		geometry_msgs::PoseArray opt_graph_msg;
 	    opt_graph_msg.header.stamp    = currPose->header.stamp;
 	    opt_graph_msg.header.frame_id = currPose->header.frame_id;

		for(int i=1 ; i<=nodeNum ; i++){
			double x = result.at<Pose2>(i).x();
			double y = result.at<Pose2>(i).y();
			double theta = result.at<Pose2>(i).theta();
			tf::Transform PoseTf;
			createTfFromXYTheta(x, y, theta, PoseTf);
			geometry_msgs::Pose pose;
			tf::poseTFToMsg(PoseTf, pose);
			opt_graph_msg.poses.push_back(pose);
		}

		opt_graph_pub.publish(opt_graph_msg);
		////////////////////////////////////////////////////////////////////////////////////////////

		kfPose = currPose; // UPDATE kfPose
		opt_flag = 1; // flag for opt_pose_msg

		// reset travel dist value //
		travel_dist_linear = 0;
		travel_dist_angular = 0;
	}

}

void hEdgeCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg)
{
	string historyNum;
	historyNum = msg->header.frame_id;
	
	if(historyNum == "1"){
		historyEdgeTime[1] = msg->header.stamp.nsec;
		Edge[1] = msg;
	}
	else{
		int num;
		num = std::stoi(historyNum);
		historyEdgeTime[num] = msg->header.stamp.nsec;
		Edge[num] = msg;
	}
}

void poseCallback(const geometry_msgs::PoseStamped::Ptr& msg)
{
    currPose = msg;

	if(opt_flag==0){createTfFromXYTheta(0.0, 0.0, 0.0, OptKfPoseTf);}
	if(pose_callback_flag==0){kfPose = currPose; pose_callback_flag = 1;}

	tf::Transform currPoseTf;
	createTfFromXYTheta(currPose->pose.position.x, currPose->pose.position.y, getYawFromMsg(currPose), currPoseTf);

	tf::Transform kfPoseTf;
	createTfFromXYTheta(kfPose->pose.position.x, kfPose->pose.position.y, getYawFromMsg(kfPose), kfPoseTf);

	tf::Transform T;
	T = kfPoseTf.inverse() * currPoseTf;

	tf::Transform optCurrPose;
	optCurrPose = OptKfPoseTf * T;

	// pack into ROS msg

	geometry_msgs::PoseStamped::Ptr opt_pose_msg;
	opt_pose_msg = boost::make_shared<geometry_msgs::PoseStamped>();

    opt_pose_msg->header.stamp    = currPose->header.stamp;
    opt_pose_msg->header.frame_id = currPose->header.frame_id; // gtsam starting point

    tf::poseTFToMsg(optCurrPose, opt_pose_msg->pose);

	opt_pose_pub.publish(opt_pose_msg);
	
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "PoseGraphBuilder");
  ros::NodeHandle n;

  ROS_INFO("Pose Graph Builder Start !");

  if(!n.getParam("/posegraph/history", history)){history = 2;}

  //set first pose node
  Pose2 priorMean(0.285, 0.0, 0.0); // prior at origin
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.0, 0.0, 0.0));
  graph.emplace_shared<PriorFactor<Pose2> >(1, priorMean, priorNoise);
  initial.insert(1, Pose2(0.285, 0.0, 0.0));

  ros::Subscriber h_edge_sub = n.subscribe("history_edge", 10, hEdgeCallback);
  ros::Subscriber pose_sub = n.subscribe("pose_stamped", 10, poseCallback);
  ros::Subscriber edge_sub = n.subscribe("edge", 10, edgeCallback);

  opt_pose_pub = n.advertise<geometry_msgs::PoseStamped>("opt_pose_stamped", 10);

  opt_graph_pub = n.advertise<geometry_msgs::PoseArray>("opt_graph", 10);

  ros::spin();

  return 0;
}
