#include <vector>
#include <iostream>
#include <utility>
#include <tuple>
#include <cmath>

#include <gazebo_msgs/ModelStates.h>
#include <pcl_ros/point_cloud.h>

#include "SarsaLearner.h"
#include "SupervisedLearner.h"

using namespace std;

typedef tuple<vector<float>, vector<int>, float> CommandStateActionQ;
static CommandStateActionQ nullTuple = make_tuple(vector<float>(), vector<int>(), -numeric_limits<float>::infinity());

class PTAMLearner : public SarsaLearner, public SupervisedLearner
{
private:
	ros::NodeHandle nh;
	ros::Subscriber gazeboModelStates_sub, pointCloud_sub;
	static pthread_mutex_t gazeboModelState_mutex, pointCloud_mutex;

	geometry_msgs::Pose robotWorldPose;
	pcl::PointCloud<pcl::PointXYZ> currentPointCloud;
	vector<CommandStateActionQ> possibleTrajectories;
	CommandStateActionQ lastBestQStateAction;
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
	CommandStateActionQ getBestSLStateAction(vector<float> lastCommand);
	vector<CommandStateActionQ> getSLActions();
	void clear();
	void getActions();
};	