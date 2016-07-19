#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <utility>
#include <tuple>

#include "PTAMLearner.h"

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <ptam_com/ptam_info.h>
#include <ptam_com/PosePointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <gazebo_msgs/ModelStates.h>

#include <tf/transform_listener.h>

#define Q_THRESH -10
using namespace std;

class JoystickNode
{
private:
	enum {A, B, X, Y, LB, RB, BACK, START, POWER, LS, RS}; //joystick buttons according to their index
	enum {LH, LV, LT, RH, RV, RT, DH, DV}; //joystick axes
	
	string MODE;
	int NUM_EPISODES, MAX_STEPS;

	geometry_msgs::PoseWithCovarianceStamped pose, cpose;
	geometry_msgs::Twist vel;
	pcl::PointCloud<pcl::PointXYZ> pointCloud, prevPointCloud;
	ptam_com::ptam_info ptamInfo;
	visualization_msgs::MarkerArray markerArray;
	vector<nav_msgs::Odometry> odomMsgs;
	visualization_msgs::Marker path;
	sensor_msgs::Joy joy;
	geometry_msgs::Pose robotWorldPose;


	string robotName;
	int state, breakCount, num_broken, num_inits, num_steps;
	vector<float> lastCommand, lastRLInput; //last command sent to the planner
	vector<vector<float> > episode;
	vector<vector<vector<float> > > episodeList;
	bool badEstimate, just_init, initialized;
	float rlRatio;
	float prevQ; 	
	float initY;
	ofstream qFile;
	
	//Subscribers
	ros::Subscriber joy_sub; 
	ros::Subscriber pose_sub,cam_pose_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber pointCloud_sub;
	ros::Subscriber ptamInfo_sub;
	ros::Subscriber vel_sub;
	ros::Subscriber plannerStatus_sub;
	ros::Subscriber gazeboModelStates_sub;
	ros::Subscriber globalPoints_sub;
	ros::Subscriber init_sub, sendCommand_sub; 

	static pthread_mutex_t pose_mutex, cpose_mutex;
	static pthread_mutex_t pointCloud_mutex;
	static pthread_mutex_t ptam_mutex;
	static pthread_mutex_t markerArray_mutex;
	static pthread_mutex_t odom_mutex;
	static pthread_mutex_t vel_mutex;
	static pthread_mutex_t plannerStatus_mutex;
	static pthread_mutex_t gazeboModelState_mutex;
	static pthread_mutex_t globalPlanner_mutex;

	// Publishers
	ros::Publisher vel_pub;
	ros::Publisher reset_pub;
	ros::Publisher planner_pub;
	ros::Publisher planner_reset_pub;
	ros::Publisher path_pub;
	ros::Publisher navdata_pub;
	ros::Publisher gazebo_state_reset_pub;
	ros::Publisher ptam_com_pub;
	ros::Publisher pose_pub, next_pose_pub;
	ros::Publisher global_planner_pub, expected_pub;
	ros::Publisher init_pub, sendCommand_pub;

	ros::NodeHandle nh;
	ros::ServiceClient posePointCloudClient, expectedPathClient;
	//tf::TransformListener listener;

	PTAMLearner learner; //Q learning agent
	
	//void Init();

//	vector<float> getPoseOrientation(geometry_msgs::Quaternion quat);
	
	//Navigation Funtions
	//void sendCommand();
	//vector<vector<float> > getTrajectories();
	//vector<float> getRandomTrajectory();
	
	//RL Funtions
//	pcl::PointCloud<pcl::PointXYZ> getPointCloud(vector<float> input);
//	tuple<vector<float>, vector<float>, float> getBestQStateAction(vector<vector<float> > inputs);
//	vector<float> getRLInput(vector<float> input, pcl::PointCloud<pcl::PointXYZ> currentPointCloud);
//	vector<float> getRLInput(vector<float> input, pcl::PointCloud<pcl::PointXYZ> currentPointCloud, pcl::PointCloud<pcl::PointXYZ> nextPointCloud);
//	bool inLimits(float x, float y);
//	geometry_msgs::PoseStamped getPoseFromInput(vector<float> input);
//	vector<pcl::PointXYZ> pointCloudIntersection(pcl::PointCloud<pcl::PointXYZ> currentPointCloud, pcl::PointCloud<pcl::PointXYZ> nextPointCloud);
	
	//Callback Funtions
	void poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr);
	void camPose(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr);
	void joyCb(const sensor_msgs::JoyPtr joyPtr);
	void odomCb(const nav_msgs::OdometryPtr odomPtr);
	void pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr);	
	void ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr);	
	void plannerStatusCb(const std_msgs::StringPtr plannerStatusPtr);	
	void gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr);
	void globalNextPoseCb(const std_msgs::Float32MultiArrayPtr arrayPtr);
	void initCb(const std_msgs::EmptyPtr emptyPtr);
	void sendCommandCb(const std_msgs::EmptyPtr emptyPtr);

public:
	JoystickNode();
	~JoystickNode();
};

