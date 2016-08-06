#include <vector>
#include <iostream>
#include <utility>
#include <tuple>

#include <gazebo_msgs/ModelStates.h>
#include <pcl_ros/point_cloud.h>

#include "SarsaLearner.h"

using namespace std;

typedef tuple<vector<float>, vector<int>, float> CommandStateActionQ;

class PTAMLearner : public SarsaLearner
{
private:
	ros::NodeHandle nh;
	ros::Subscriber gazeboModelStates_sub, pointCloud_sub;
	ros::ServiceClient SLclient;
	static pthread_mutex_t gazeboModelState_mutex, pointCloud_mutex;

	geometry_msgs::Pose robotWorldPose;
	pcl::PointCloud<pcl::PointXYZ> currentPointCloud;
	
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
	CommandStateActionQ getSLClosestAngleStateAction(float nextAngle);
	CommandStateActionQ getSLRandomStateAction();
};	