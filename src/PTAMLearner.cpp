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
	cout << "Best Q start" << endl;
	vector<tuple<vector<float>, vector<float>, float> > result;
	int index;
	float maxQ = -numeric_limits<float>::infinity();  //init max Q to negative infinity
	float Q;
	vector<float> rl_input;
	vector<vector<float> > inputs = Helper::getTrajectories();
	cout << "Inside Best Q  - initial shit done" << endl;
	int adder = 0; //if going into the continue loop use this extra variable

	for(vector<vector<float> >::iterator inp = inputs.begin(); inp!=inputs.end(); ++inp)
	{	
		cout << "Inside Best Q  - inside the for loop" << endl;
		if(lastCommand.size() and (!(lastCommand[12]+(*inp)[12]) and !(lastCommand[5] + (*inp)[5]))){
			cout << "Inside Best Q  - inside the if statement" << endl;
			adder += 1;
			continue;
		}
		result.push_back(getAction(*inp, currentPointCloud));
		cout << "Inside Best Q  - results pushed" << endl;
		Q = get<2>(result.back());
		cout << "Inside Best Q  - Current Q value " << Q << endl;
		cout << "Inside Best Q  - Current Q value MAX " << maxQ << endl;
		cout << "Inside Best Q  - value of index " << ((inp - inputs.begin()) - adder )<< endl;
		cout << "Inside Best Q  - THe adder " << ((inp - inputs.begin()) - adder )<< endl;

		//if(maxQ<Q and Q!=0)//take best Q
		if(maxQ<Q)
		{
			cout << "Inside Best Q  - inside the other if" << endl;
			maxQ = Q;
			cout << "Inside Best Q  - inside the other if Part 2" << endl;
			index = (inp - inputs.begin()) - adder;
			cout << "Inside Best Q  - inside the other if part party 3" << endl;
		}
		cout << "Inside Best Q  - for loop ends" << endl;

	}

	cout << "Inside Best Q  - out of the for loop" << endl;

	if(maxQ == -numeric_limits<float>::infinity()){
		cout << "Inside Best Q  - inside the last if start" << endl;
		index = rand()%inputs.size();
		cout << "Inside Best Q  - inside the last if end" << endl;
	}
	cout << "Inside Best Q  - before returning" << endl;
	cerr << "size of result "<< result.size() << endl;
	cerr << "The index is"<< index << endl;

	int x = 0;
	for ( const auto& i : result ) {
		cout<<"Q value number  :: "<< x << " :: "<< get<2>(i) <<endl;
		cout<<"testing with the indices :: "<< x << " :: "<< get<2>(result[x]) <<endl;
		x++;

  //cout << get<0>(i) << get<1>(i) << get<2>(i) << endl;
}
	cout<<"Q value number Index - 1 :: "<<index -1 << " :: "<< get<2>(result[index - 1]) <<endl;
	cout<<"Q value number Index  :: "<<index << " :: "<< get<2>(result[index]) <<endl;
	return result[index];
	cout << "Best Q  end" << endl;
}

tuple<vector<float>, vector<float>, float> PTAMLearner::getRandomStateAction(pcl::PointCloud<pcl::PointXYZ> currentPointCloud)
{
	vector<vector<float> > trajectories = Helper::getTrajectories();	
	return getAction(trajectories[rand()%trajectories.size()], currentPointCloud);
}
