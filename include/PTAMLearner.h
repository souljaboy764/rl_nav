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

	ros::NodeHandle nh;
	ros::ServiceClient posePointCloudClient;

public:	
	PTAMLearner();

/*	pcl::PointCloud<pcl::PointXYZ> getPointCloud(vector<float> input);//, geometry_msgs::PoseWithCovarianceStamped pose);
	vector<float> getRLInput(vector<float> input, pcl::PointCloud<pcl::PointXYZ> currentPointCloud);
	tuple<vector<float>, vector<float>, float> getAction(vector<float> input, pcl::PointCloud<pcl::PointXYZ> currentPointCloud);
	tuple<vector<float>, vector<float>, float> getRandomStateAction(pcl::PointCloud<pcl::PointXYZ> currentPointCloud);
	tuple<vector<float>, vector<float>, float> getBestQStateAction(vector<float> lastRLInput, pcl::PointCloud<pcl::PointXYZ> currentPointCloud);
*/
	tuple<vector<float>, vector<float>, float> getAction(vector<float> input);
	tuple<vector<float>, vector<float>, float> getRandomStateAction();
	tuple<vector<float>, vector<float>, float> getBestQStateAction(vector<float> lastRLInput);
	tuple<vector<float>, vector<float>, float> getEpsilonGreedyStateAction(float epsilon, vector<float> lastRLInput);

/*	void poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr);
	void ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr);	
*/};	