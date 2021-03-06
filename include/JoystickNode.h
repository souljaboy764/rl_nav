#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <utility>
#include <tuple>

#include "PTAMLearner.h"

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <ptam_com/ptam_info.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class JoystickNode
{
private:
	enum {A, B, X, Y, LB, RB, BACK, START, POWER, LS, RS}; //joystick buttons according to their index
	enum {LH, LV, LT, RH, RV, RT, DH, DV}; //joystick axes
	
	ros::NodeHandle nh;

	//Subscribers
	ros::Subscriber joy_sub,
					pose_sub,
					pointCloud_sub,
					ptamInfo_sub,
					plannerStatus_sub,
					gazeboModelStates_sub,
					globalPoints_sub,
					init_sub,
					sendCommand_sub,
					ptamStart_sub,
					waypoint_sub,
					goal_sub;

	// Publishers
	ros::Publisher  vel_pub,
					planner_pub,
					planner_reset_pub,
					gazebo_state_reset_pub,
					ptam_com_pub,
					pose_pub,
					next_pose_pub,
					next_pc_pub,
					expected_pub,
					ptam_path_pub,
					gazebo_path_pub,
					init_pub,
					sendCommand_pub,
					safe_poses_pub,
					unsafe_poses_pub,
					gazebo_pose_pub,
					ptam_pc_pub,
					odom_reset_pub,
					goal_pub;

	ros::ServiceClient expectedPathClient;
	
	//Mutex locks	
	static pthread_mutex_t pose_mutex;
	static pthread_mutex_t pointCloud_mutex;
	static pthread_mutex_t ptamInfo_mutex;
	static pthread_mutex_t plannerStatus_mutex;
	static pthread_mutex_t gazeboModelState_mutex;
	static pthread_mutex_t globalPlanner_mutex;
	
	geometry_msgs::PoseStamped pose, goalPose;
	geometry_msgs::Twist vel;
	pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
	//ptam_com::ptam_info ptamInfo;
	std_msgs::Bool ptamInfo;
	sensor_msgs::Joy joy;
	gazebo_msgs::ModelState initState;
	geometry_msgs::Pose robotWorldPose;
	visualization_msgs::Marker vslam_path, gazebo_path;
	geometry_msgs::Pose  waypointPose;

	tf::TransformBroadcaster tfBroadcaster;
		
	string MODE;
	int MAX_EPISODES, MAX_STEPS, MAP;
	float Q_THRESH;
	double INIT_ANGLE;
	int state, breakCount, num_broken, num_steps, num_episodes;
	geometry_msgs::PoseStamped lastPose; //last pose sent to the planner
	vector<vector<int> > episode;
	vector<vector<vector<int> > > episodeList;
	bool badEstimate, just_init, initialized;
	float rlRatio;
	float prevQ; 	
	float initY, initX, initZ, initYaw;
	ofstream qFile;
	bool left, right, up, down;
	float vel_scale;
	int num;

	PTAMLearner learner; //Q learning agent

	MoveBaseClient *actionClient; //actionlib client to handle goals
	
	//Callback Funtions
	void poseCb(const geometry_msgs::PoseStampedPtr posePtr);
	void joyCb(const sensor_msgs::JoyPtr joyPtr);
	void pointCloudCb(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr);	
	//void ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr);	
	void ptamInfoCb(const std_msgs::BoolPtr ptamInfoPtr);	
	void plannerStatusCb(const std_msgs::StringPtr plannerStatusPtr);	
	void gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr);
	void globalNextPoseCb(const geometry_msgs::PoseStampedPtr nextPosePtr);
	void initCb(const std_msgs::Empty empty);
	void sendCommandCb(const std_msgs::Empty empty);
	void ptamStartedCb(const std_msgs::EmptyPtr emptyPtr);
	void waypointCb(const geometry_msgs::PoseStampedPtr waypointPosePtr);
	void goalCb(const geometry_msgs::PoseStampedPtr goalPosePtr);

public:
	JoystickNode();
	~JoystickNode();
};