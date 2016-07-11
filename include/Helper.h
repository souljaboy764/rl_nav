#include <algorithm>
#include <limits>
#include <numeric>
#include <functional>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <utility>
#include <tuple>

#include <cmath>
#include <ctime>
#include <cstdlib>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <ptam_com/ptam_info.h>
#include <ptam_com/PosePointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <gazebo_msgs/ModelStates.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

using namespace std;

const long double PI = 3.141592653589793238L;

#define min(a,b) (a<b)?a:b
#define max(a,b) (a>b)?a:b

class Helper
{
	struct pointEqComparer
	{
		bool operator()(const pcl::PointXYZ &p_left, const pcl::PointXYZ &p_right)
		{
			return p_left.x == p_right.x and p_left.y == p_right.y and p_left.z == p_right.z;
		}
	};

	struct pointLtComparer
	{
		bool operator()(const pcl::PointXYZ &p_left, const pcl::PointXYZ &p_right)
		{
			return p_left.x < p_right.x and p_left.y < p_right.y and p_left.z < p_right.z;
		}
	};

	static pthread_mutex_t pose_mutex, info_mutex, gazeboModelState_mutex;

	static geometry_msgs::PoseWithCovarianceStamped pose;
	static ptam_com::ptam_info ptamInfo;
	static geometry_msgs::Pose robotWorldPose;

	ros::NodeHandle nh;
	ros::Subscriber pose_sub, info_sub, gazeboModelStates_sub;
	static ros::ServiceClient posePointCloudClient;

	void poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr);
	void ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr);	
	void gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr);

public:
	Helper();
	static pcl::PointCloud<pcl::PointXYZ> getPointCloud(vector<float> input);
	static vector<float> getRLInput(vector<float> input, pcl::PointCloud<pcl::PointXYZ> currentPointCloud);
	static vector<double> getPoseOrientation(geometry_msgs::Quaternion quat);
	static geometry_msgs::PoseStamped getPoseFromInput(vector<float> input, geometry_msgs::PoseWithCovarianceStamped pose);
	static vector<pcl::PointXYZ> pointCloudIntersection(pcl::PointCloud<pcl::PointXYZ> currentPointCloud, pcl::PointCloud<pcl::PointXYZ> nextPointCloud);
	static bool inLimits(float x, float y);
	static vector<vector<float> > getTrajectories();
	static geometry_msgs::Pose getRobotWorldPose();
	static void saveFeatureExpectation(vector<vector<vector<float> > > episodeList);
	
};