#include <vector>
#include <iostream>
#include <utility>
#include <tuple>

#include <ptam_com/ptam_info.h>
#include <gazebo_msgs/ModelStates.h>
#include <pcl_ros/point_cloud.h>

#include "SarsaLearner.h"

using namespace std;

typedef tuple<vector<float>, vector<unsigned int>, float> CommandStateActionQ;

class PTAMLearner : public SarsaLearner
{
private:
	ros::NodeHandle nh;
	ros::Subscriber info_sub, gazeboModelStates_sub, pointCloud_sub;
	static pthread_mutex_t info_mutex, gazeboModelState_mutex, pointCloud_mutex;

	ptam_com::ptam_info ptamInfo;
	geometry_msgs::Pose robotWorldPose;
	pcl::PointCloud<pcl::PointXYZ> currentPointCloud;
	
	void ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr);	
	void gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr);
	void pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr);	

public:	
	PTAMLearner();

	CommandStateActionQ getAction(vector<float> input);
	CommandStateActionQ getRandomStateAction();
	CommandStateActionQ getThresholdedRandomStateAction(float qThreshold, int maxIters);
	CommandStateActionQ getBestQStateAction(vector<float> lastCommand);
	CommandStateActionQ getEpsilonGreedyStateAction(float epsilon, vector<float> lastCommand);
	CommandStateActionQ getThresholdedClosestAngleStateAction(float qThreshold, float nextAngle, vector<float> lastCommand);
};	