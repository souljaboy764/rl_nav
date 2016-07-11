#include "Helper.h"
#include "PTAMLearner.h"

#include <geometry_msgs/Pose.h>

using namespace std;

pthread_mutex_t PTAMLearner::pose_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PTAMLearner::info_mutex = PTHREAD_MUTEX_INITIALIZER;

PTAMLearner::PTAMLearner()
{
	srand (time(NULL));
}

tuple<vector<float>, vector<float>, float> PTAMLearner::getAction(vector<float> input, pcl::PointCloud<pcl::PointXYZ> currentPointCloud)
{
	vector<float> rl_input = Helper::getRLInput(input, currentPointCloud);
	return make_tuple(input,rl_input,getQ(rl_input));
}


tuple<vector<float>, vector<float>, float> PTAMLearner::getBestQStateAction(vector<float> lastCommand, pcl::PointCloud<pcl::PointXYZ> currentPointCloud)
{
	vector<tuple<vector<float>, vector<float>, float> > result;
	int index;
	float maxQ = -numeric_limits<float>::infinity(), Q;
	vector<float> rl_input;
	vector<vector<float> > inputs = Helper::getTrajectories();
	for(vector<vector<float> >::iterator inp = inputs.begin(); inp!=inputs.end(); ++inp)
	{
		if(lastCommand.size() and (!(lastCommand[12]+(*inp)[12]) and !(lastCommand[5] + (*inp)[5])))
			continue;
		result.push_back(getAction(*inp, currentPointCloud));
		Q = get<2>(result.back());
		if(maxQ<Q and Q!=0)//take best Q
		{
			maxQ = Q;
			index = inp - inputs.begin();
		}
	}

	if(maxQ == -numeric_limits<float>::infinity())
		index = rand()%inputs.size();
	return result[index];
}

tuple<vector<float>, vector<float>, float> PTAMLearner::getRandomStateAction(pcl::PointCloud<pcl::PointXYZ> currentPointCloud)
{
	geometry_msgs::Pose robotWorldPose = Helper::getRobotWorldPose();
	vector<double> angles = Helper::getPoseOrientation(robotWorldPose.orientation);
	float angle_step = PI/20.0;
	int num = 3, max_num = 7;
	float angle = ((rand()%max_num) - num)* angle_step;
	float dir = (rand()%2)?1.0:-1.0;
	
	vector<float> inp = {0.0,0.0,0.0, 
						 dir*cos(angle), sin(angle), dir*tan(angle),
						 0.0,0.0,0.0,0.0,0.0,0.0,
						 dir,0.0,1.5};
	float x = robotWorldPose.position.x + dir*cos(angles[2] + dir*angle), y = robotWorldPose.position.y + dir*sin(angles[2] + dir*angle);
	if(Helper::inLimits(x,y))
	{
		return getAction(inp, currentPointCloud);
		/*rl_input = getRLInput(inp, currentPointCloud);
		Q = getQ(rl_input);
		return make_tuple(inp,rl_input,Q);*/
	}
		
	return getRandomStateAction(currentPointCloud);
}
