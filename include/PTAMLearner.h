#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <utility>
#include <tuple>

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <ptam_com/ptam_info.h>

#include <ros/ros.h>

#include "SarsaLearner.h"

using namespace std;

class PTAMLearner : public SarsaLearner
{
private:
	static pthread_mutex_t pose_mutex, info_mutex;

	static geometry_msgs::PoseWithCovarianceStamped pose;
	static ptam_com::ptam_info ptamInfo;

	ros::NodeHandle nh;
	ros::ServiceClient posePointCloudClient;

public:	
	PTAMLearner();

/*	pcl::PointCloud<pcl::PointXYZ> getPointCloud(vector<float> input);//, geometry_msgs::PoseWithCovarianceStamped pose);
	vector<float> getRLInput(vector<float> input, pcl::PointCloud<pcl::PointXYZ> currentPointCloud);
*/	tuple<vector<float>, vector<float>, float> getAction(vector<float> input, pcl::PointCloud<pcl::PointXYZ> currentPointCloud);
	tuple<vector<float>, vector<float>, float> getRandomStateAction(pcl::PointCloud<pcl::PointXYZ> currentPointCloud);
	tuple<vector<float>, vector<float>, float> getBestQStateAction(vector<float> lastRLInput, pcl::PointCloud<pcl::PointXYZ> currentPointCloud);

/*	void poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr);
	void ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr);	
*/};	