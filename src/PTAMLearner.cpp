#include <fstream>

#include "Helper.h"
#include "PTAMLearner.h"

#include <ros/package.h>

using namespace std;

pthread_mutex_t PTAMLearner::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PTAMLearner::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
int PTAMLearner::MAX_POINT_OVERLAP;

PTAMLearner::PTAMLearner()
{
	pointCloud_sub = nh.subscribe("/vslam/frame_points", 100, &PTAMLearner::pointCloudCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &PTAMLearner::gazeboModelStatesCb, this);
	srand (time(NULL));
	ros::NodeHandle p_nh("~");
	p_nh.getParam("MAX_POINT_OVERLAP", MAX_POINT_OVERLAP);
	lastBestQStateAction = nullTuple;
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

//convert all the possible trajectories into state-action format and store them
void PTAMLearner::getActions()
{
	if(!possibleTrajectories.size())
		for(auto trajectory : Helper::getTrajectories())
			possibleTrajectories.push_back(getAction(trajectory));
}

//convert a given bernstein input relative to the robot camera frame into a state-action pair and get the Q value
CommandStateActionQ PTAMLearner::getAction(vector<float> input)
{
	vector<int> rl_input;
	pcl::PointCloud<pcl::PointXYZ> nextPointCloud = Helper::getPCLPointCloudAtPosition(input);
	
	pthread_mutex_lock(&pointCloud_mutex);
	vector<pcl::PointXYZ> commonPoints = Helper::pointCloudIntersection(currentPointCloud,nextPointCloud);
	pthread_mutex_unlock(&pointCloud_mutex);

	float dir = input[12], del_heading = atan(input[5]);
	//RL params
	rl_input.push_back((int)(dir==1)?1:0);
	if(fabs(del_heading)*180.0/PI > 30)
		rl_input.push_back(19);
	else
		rl_input.push_back((int) 19*fabs((del_heading)*180.0/(30*PI)));

	rl_input.push_back((int) min(19,commonPoints.size()*20.0/((float)MAX_POINT_OVERLAP)));

	return make_tuple(input,rl_input,getQ(rl_input));
}

//returns the state-action with the highest Q value except the previously executed command so that it doesn't redo it
CommandStateActionQ PTAMLearner::getBestQStateAction(vector<float> lastCommand)
{
	cout<<"getBestQStateAction start"<<endl;
	if(lastBestQStateAction!=nullTuple)
		return lastBestQStateAction;
	cout<<"lastBestQStateAction not null"<<endl;
	vector<CommandStateActionQ> result;
	int index=-1;
	float maxQ = -numeric_limits<float>::infinity();  //init max Q to negative infinity
	float Q;
	cout<<"getting actions"<<endl;
	getActions();
	//vector<vector<float> > inputs = Helper::getTrajectories();
	cout<<"evaluating actions"<<endl;
	for(auto input : possibleTrajectories)
	{	
		vector<float> inp = get<0>(input);
		if( lastCommand.size() and 
			!(lastCommand[12]+inp[12]) and 
			!(lastCommand[5] + inp[5]) and 
			!(fabs(lastCommand[5]) - fabs(inp[5])))
			continue;
		result.push_back(input);
		Q = get<2>(input);
		if(maxQ<Q)
		{
			maxQ = Q;
			index = result.size()-1;
		}
	}
	cout<<"finished evaluating actions"<<endl;
	if(maxQ == -numeric_limits<float>::infinity() or !result.size())
	{
		cout<<"result size zero"<<endl;
		if(possibleTrajectories.size())
		{
			cout<<"returning random from possibleTrajectories of size "<<possibleTrajectories.size()<<endl;
			lastBestQStateAction = possibleTrajectories[rand()%possibleTrajectories.size()];
		}
		else
		{
			cout<<"returning random"<<endl;
			lastBestQStateAction = nullTuple;
			return getRandomStateAction();
		}
	}	

		
	else if(index!=-1 and result.size())
	{
		cout<<"setting lastBestQStateAction"<<endl;
		lastBestQStateAction = result[index];
	}
	cout<<"getBestQStateAction end"<<endl;
	return lastBestQStateAction;
}

//epsilon greedy policy
CommandStateActionQ PTAMLearner::getEpsilonGreedyStateAction(float epsilon, vector<float> lastCommand)
{
	if((rand() % 100) < epsilon)
		return getBestQStateAction(lastCommand);
	else
		return getRandomStateAction();
}

//random policy
CommandStateActionQ PTAMLearner::getRandomStateAction()
{
	vector<vector<float> > trajectories = Helper::getTrajectories();	
	cout<<trajectories.size()<<endl;
	return getAction(trajectories[rand()%trajectories.size()]);
}

//thresholded random, i.e. returns an action with a Q value higher than qThreshold
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
	getActions();
	
	for(auto input : possibleTrajectories)
		if(get<2>(input) > qThreshold)
			potentialInputs.push_back(input);
	  
	if(potentialInputs.size()<1)
		return getBestQStateAction(lastCommand);
	else
	{
		cout<<"THRESHOLDED SIZE "<<potentialInputs.size()<<endl;
		CommandStateActionQ result = potentialInputs[rand()%potentialInputs.size()];
		//need to change this to take the SLAM pose
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

//get supervised learner actions
vector<CommandStateActionQ> PTAMLearner::getSLActions()
{
	if(!slValid)
		return vector<CommandStateActionQ>();

	vector<CommandStateActionQ> potentialInputs;
	getActions();
	for(auto input : possibleTrajectories)
		if(!predict(get<1>(input)))
			potentialInputs.push_back(input);
	return potentialInputs;
}	

////get supervised learner actions with angle closest to nextAngle
CommandStateActionQ PTAMLearner::getSLClosestAngleStateAction(float nextAngle)
{
	if(!slValid)
		return getRandomStateAction();

	vector<CommandStateActionQ> potentialInputs = getSLActions();
	float currentAngle = Helper::getPoseOrientation(robotWorldPose.orientation)[2], min_diff = numeric_limits<float>::infinity(), angle_diff;
		
	CommandStateActionQ result = getRandomStateAction();
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

CommandStateActionQ PTAMLearner::getSLRandomStateAction()
{
	if(!slValid)
		return getRandomStateAction();

	vector<CommandStateActionQ> potentialInputs = getSLActions();
	
	if(potentialInputs.size())
		return potentialInputs[rand()%potentialInputs.size()];
	else
		return getRandomStateAction();
}

CommandStateActionQ PTAMLearner::getBestSLStateAction(vector<float> lastCommand)
{
	if(!slValid)
		return getBestQStateAction(lastCommand);

	vector<CommandStateActionQ> result;
	vector<CommandStateActionQ> potentialInputs = getSLActions();
	int index;
	float maxDist = -numeric_limits<float>::infinity();  //init max Distance to -infinity
	float dist;
	
	for(auto inp : potentialInputs)
	{	
		vector<float> command = get<0>(inp);
		if(lastCommand.size() and 
			(	!(lastCommand[12]+ command[12]) and 
				!(lastCommand[5] + command[5]) and 
				!(19*fabs(lastCommand[5])/30.0 - 19*fabs(command[5])/30.0)))
			continue;
		result.push_back(inp);
		dist = distance(get<1>(result.back()));
		if(maxDist<dist)
		{
			maxDist = dist;
			index = result.size()-1;
		}
	}

	if(maxDist == -numeric_limits<float>::infinity())
		return getSLRandomStateAction();
	
	return result[index];
}

//lastBestQStateAction is stored so that it doesn't need to be recomputed each time
void PTAMLearner::clear()
{
	possibleTrajectories.clear();
	lastBestQStateAction = nullTuple;
}