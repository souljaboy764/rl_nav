#include "Helper.h"
#include "PTAMLearner.h"

#include <geometry_msgs/Pose.h>

using namespace std;

PTAMLearner::PTAMLearner()
{
	srand (time(NULL));
}

tuple<vector<float>, vector<float>, float> PTAMLearner::getAction(vector<float> input)//, pcl::PointCloud<pcl::PointXYZ> currentPointCloud)
{
	//vector<float> rl_input = Helper::getRLInput(input, currentPointCloud);
	vector<float> rl_input = Helper::getRLInput(input);
	return make_tuple(input,rl_input,getQ(rl_input));
}


tuple<vector<float>, vector<float>, float> PTAMLearner::getBestQStateAction(vector<float> lastCommand)//, pcl::PointCloud<pcl::PointXYZ> currentPointCloud)
{
	vector<tuple<vector<float>, vector<float>, float> > result;
	int index;
	float maxQ = -numeric_limits<float>::infinity();  //init max Q to negative infinity
	float Q;
	vector<float> rl_input;
	vector<vector<float> > inputs = Helper::getTrajectories();

	for(vector<vector<float> >::iterator inp = inputs.begin(); inp!=inputs.end(); ++inp)
	{	
		if(lastCommand.size() and (!(lastCommand[12]+(*inp)[12]) and !(lastCommand[5] + (*inp)[5]))) //condition to prevent opposite actions
			continue;
		result.push_back(getAction(*inp));//, currentPointCloud));
		Q = get<2>(result.back());
		if(maxQ<Q)
		{
			maxQ = Q;
			index = result.size()-1;
		}
	}

	if(maxQ == -numeric_limits<float>::infinity())
		index = rand()%inputs.size();

	return result[index];
}

tuple<vector<float>, vector<float>, float> PTAMLearner::getEpsilonGreedyStateAction(float epsilon, vector<float> lastCommand)//, pcl::PointCloud<pcl::PointXYZ> currentPointCloud)
{
	if((rand() % 100) < epsilon)
		return getBestQStateAction(lastCommand);//, currentPointCloud);
	else
		return getRandomStateAction();//currentPointCloud);
}

tuple<vector<float>, vector<float>, float> PTAMLearner::getRandomStateAction()//pcl::PointCloud<pcl::PointXYZ> currentPointCloud)
{
	vector<vector<float> > trajectories = Helper::getTrajectories();	
	return getAction(trajectories[rand()%trajectories.size()]);//, currentPointCloud);
}
