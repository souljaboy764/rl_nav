#include "Helper.h"
#include "PTAMLearner.h"

using namespace std;

pthread_mutex_t PTAMLearner::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PTAMLearner::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;

PTAMLearner::PTAMLearner()
{
	pointCloud_sub = nh.subscribe("/vslam/frame_points", 100, &PTAMLearner::pointCloudCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &PTAMLearner::gazeboModelStatesCb, this);
	srand (time(NULL));
}

void PTAMLearner::gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr)
{
	pthread_mutex_lock(&gazeboModelState_mutex);	
	robotWorldPose = modelStatesPtr->pose.back();
	pthread_mutex_unlock(&gazeboModelState_mutex);
}

void PTAMLearner::pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr)	
{
	pthread_mutex_lock(&pointCloud_mutex);
	currentPointCloud = *pointCloudPtr;
	pthread_mutex_unlock(&pointCloud_mutex);
}


CommandStateActionQ PTAMLearner::getAction(vector<float> input)
{
	vector<unsigned int> rl_input;
	pcl::PointCloud<pcl::PointXYZ> nextPointCloud = Helper::getPointCloudAtPosition(input);
	
	pthread_mutex_lock(&pointCloud_mutex);
	vector<pcl::PointXYZ> commonPoints = Helper::pointCloudIntersection(currentPointCloud,nextPointCloud);
	pthread_mutex_unlock(&pointCloud_mutex);

	float dir = input[12], del_heading = atan(input[5]);
	//RL params
	rl_input.push_back((unsigned int)(dir==1)?2:1);
	if(fabs(del_heading)*180.0/PI > 30)
		rl_input.push_back(20);
	else
		rl_input.push_back((unsigned int) 20*fabs((del_heading)*180.0/(30*PI)));

	rl_input.push_back((unsigned int) min(20,commonPoints.size()/30.0));

	return make_tuple(input,rl_input,getQ(rl_input));
}


CommandStateActionQ PTAMLearner::getBestQStateAction(vector<float> lastCommand)
{
	vector<CommandStateActionQ> result;
	int index;
	float maxQ = -numeric_limits<float>::infinity();  //init max Q to negative infinity
	float Q;
	vector<vector<float> > inputs = Helper::getTrajectories();

	for(vector<vector<float> >::iterator inp = inputs.begin(); inp!=inputs.end(); ++inp)
	{	
		if(lastCommand.size() and (!(lastCommand[12]+(*inp)[12]) and !(lastCommand[5] + (*inp)[5]))) //condition to prevent opposite actions
			continue;
		result.push_back(getAction(*inp));
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

CommandStateActionQ PTAMLearner::getEpsilonGreedyStateAction(float epsilon, vector<float> lastCommand)
{
	if((rand() % 100) < epsilon)
		return getBestQStateAction(lastCommand);
	else
		return getRandomStateAction();
}

CommandStateActionQ PTAMLearner::getRandomStateAction()
{
	vector<vector<float> > trajectories = Helper::getTrajectories();	
	return getAction(trajectories[rand()%trajectories.size()]);
}

CommandStateActionQ PTAMLearner::getThresholdedRandomStateAction(float qThreshold, int maxIters)
{
	CommandStateActionQ result;

	do 
	{
		result = getRandomStateAction();
		maxIters--;
	} while(get<2>(result) < qThreshold and maxIters>=0);

	return result;
}

//Thresholded aligning with global path
CommandStateActionQ PTAMLearner::getThresholdedClosestAngleStateAction(float qThreshold, float nextAngle, vector<float> lastCommand)
{
	vector<CommandStateActionQ> potentialInputs;
	for(auto trajectory : Helper::getTrajectories())
	{
		CommandStateActionQ input = getAction(trajectory);
		if(get<2>(input) > qThreshold)
			potentialInputs.push_back(input);
	}
	
	if(potentialInputs.size()<1)
		return getBestQStateAction(lastCommand);
	else
	{
		CommandStateActionQ result = potentialInputs[rand()%potentialInputs.size()];
		float currentAngle = Helper::getPoseOrientation(robotWorldPose.orientation)[2], min_diff = numeric_limits<float>::infinity(), angle_diff;
		
		for(auto input : potentialInputs)
		{	
			vector<float> command = get<0>(input);
			angle_diff = fabs(nextAngle - (currentAngle + command[12]*atan(command[5])));
			if(angle_diff<=min_diff)
			{
				min_diff = angle_diff;
				result = input;
			}
		}
		return result;
	}
}