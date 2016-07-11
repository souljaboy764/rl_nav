#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <exception>

#include "SarsaLearner.h"

SarsaLearner::SarsaLearner()
{
	// Variables for reading values
	unsigned int stateDir;
	unsigned int stateHead;
	unsigned int stateFOV;
	//unsigned int statePFOV;
	float qValue;
	// Input file
	ifstream qMatFile("qMatData.txt");
	ifstream wFile("wData.txt");

	// Check for failure
	if(qMatFile == NULL)
	{
		cout<<"Invalid Q file"<<endl;
		qValid = false;
	}
	else 
	{
		qValid = true;
		for(unsigned int i = 0; i<STATE_DIR_MAX; i++)
			for(unsigned int j = 0; j<STATE_HEAD_MAX; j++)
				for(unsigned int k = 0; k<STATE_FOV_MAX; k++)
					{
						qMatFile >> stateDir >> stateHead >> stateFOV >> qValue;
						qMatrix[stateDir][stateHead][stateFOV] = (qValue>0)? 0.0 : qValue;
					}
	}

	
	if(wFile == NULL)
	{
		cout<<"Invalid w file"<<endl;
		w = vector<float>(NUM_FEATURES_SA,0);
	}
	else 
	{
		w = vector<float>(NUM_FEATURES_SA);
		for(unsigned int i = 0; i<NUM_FEATURES_SA; i++)
			wFile >> w[i];
	}

	// Close
	qMatFile.close();
	wFile.close();

}

SarsaLearner::~SarsaLearner()
{
	// Output file
	ofstream qMatFile("qMatData.txt");
	ofstream wFile("wData.txt");

	float qValue;
	// Check for failure
	if(qMatFile != NULL)
	{
		cout<<"Writing to File"<<endl;
		for(unsigned int stateDir = 0; stateDir<STATE_DIR_MAX; stateDir++)
			for(unsigned int stateHead = 0; stateHead<STATE_HEAD_MAX; stateHead++)
				for(unsigned int stateFOV = 0; stateFOV<STATE_FOV_MAX; stateFOV++)
						qMatFile << stateDir << " " << stateHead << " " << stateFOV << " " << qMatrix[stateDir][stateHead][stateFOV] << endl;
	}

	if(wFile != NULL)
	{
		for(unsigned int i = 0; i<NUM_FEATURES_SA; i++)
			wFile << w[i] << " ";
		wFile << endl;
	}

	// Close
	qMatFile.close();
	wFile.close();
}

// Function to return Q value
float SarsaLearner::getQ(vector<float> stateAction)
{
	if(!qValid)
		return 0.0;

	// Discretize state
	vector<unsigned int> discreteStateAction = discretizeState(stateAction);
	
	if(!discreteStateAction.size())
		return 0.0;
	
	unsigned int stateDir = discreteStateAction[0];
	unsigned int stateHead = discreteStateAction[1];
	unsigned int stateFOV = discreteStateAction[2];
//	unsigned int statePFOV = discreteStateAction[3];
	// Get q value
	return qMatrix[stateDir][stateHead][stateFOV];
}

/*float SarsaLearner::getReward(vector<float> stateAction)
{
	int rew_1 = 0, rew_2 = 0;

	if(stateAction[2] < 5)
		rew_1 = -10;
	else if(stateAction[2]<20)
		rew_1 = -10 + (stateAction[2] - 5)*(stateAction[2] - 5)/22.5;

	rew_2 -= fabs(stateAction[1])/6.0;

	return rew_1 + rew_2;
}*/

float SarsaLearner::getReward(vector<float> stateAction)
{
	vector<float> phi;
	if(stateAction[0]-1)
	{
		phi.push_back(0);
		phi.push_back(1);
	}
	else
	{
		phi.push_back(1);
		phi.push_back(0);
	}	
	phi.push_back(stateAction[1]/30.0);
	phi.push_back(stateAction[2]/30.0);
	phi.push_back(1 - stateAction[3]/2.0);
	return inner_product(phi.begin(), phi.end(), w.begin(), 0.0);
}

// Function to update Q value
void SarsaLearner::updateQ(vector<float> stateAction, vector<float> nextStateAction)
{
	if(!qValid)
		return;

	// Discretize state
	vector<unsigned int> nextDiscreteStateAction=discretizeState(nextStateAction);
	if(!nextDiscreteStateAction.size())
		return;

	unsigned int nextStateDir = nextDiscreteStateAction[0];
	unsigned int nextStateHead = nextDiscreteStateAction[1];
	unsigned int nextStateFOV = nextDiscreteStateAction[2];
	
	// Get q value
	float qNext = qMatrix[nextStateDir][nextStateHead][nextStateFOV];

	updateQ(stateAction, qNext);
}

void SarsaLearner::updateQ(vector<float> stateAction, float qNext)
{
	if(!qValid)
		return;

	vector<unsigned int> discreteStateAction = discretizeState(stateAction);
	if(!discreteStateAction.size())
		return;

	unsigned int stateDir = discreteStateAction[0];
	unsigned int stateHead = discreteStateAction[1];
	unsigned int stateFOV = discreteStateAction[2];

	float Q = qMatrix[stateDir][stateHead][stateFOV];

	// SARSA update
	qMatrix[stateDir][stateHead][stateFOV] += SARSA_ALPHA * (getReward(stateAction) + SARSA_GAMMA * qNext - Q);
}

void SarsaLearner::episodeUpdate(vector<vector<vector<float> > > episodeList)
{
	for(vector<vector<vector<float> > >::iterator episode = episodeList.begin(); episode!=episodeList.end(); ++episode)
	{
		int i=0;
		for(vector<vector<float> >::iterator rlStep = episode->begin(); rlStep!=episode->end()-1; ++rlStep)
			updateQ(*rlStep, *next(rlStep));
		updateQ(episode->back(), 0);	
	}
	cout<<endl;
}
// Discretize state
vector<unsigned int> SarsaLearner::discretizeState(vector<float> stateAction)
{
	float dir = stateAction[0];
	float head = stateAction[1];
	float fov = stateAction[2];
	//float pfov = stateAction[3];

	vector <unsigned int> discreteStateAction;

	// Test against limits
	if(	dir < MIN_DIR || dir > MAX_DIR ||
		head < MIN_HEAD || head > MAX_HEAD ||
		fov < MIN_FOV || fov > MAX_FOV)// || 
		//pfov < MIN_PFOV || pfov > MAX_PFOV)
	{
		return discreteStateAction;
	}	

	// Discretize using offset and factor
	discreteStateAction.push_back((unsigned int) (dir + OFFSET_DIR) / DISC_DIR);
	discreteStateAction.push_back((unsigned int) (head + OFFSET_HEAD) / DISC_HEAD);
	discreteStateAction.push_back((unsigned int) (fov + OFFSET_FOV) / DISC_FOV);
//	discreteStateAction.push_back((unsigned int) (pfov + OFFSET_PFOV) / DISC_PFOV);

	// Correct if limits crossed
	discreteStateAction[0] = (discreteStateAction[0] > DISC_DIR_MAX) ? DISC_DIR_MAX : discreteStateAction[0];
	discreteStateAction[1] = (discreteStateAction[1] > DISC_HEAD_MAX) ? DISC_HEAD_MAX : discreteStateAction[1];
	discreteStateAction[2] = (discreteStateAction[2] > DISC_FOV_MAX) ? DISC_FOV_MAX : discreteStateAction[2];
//	discreteStateAction[3] = (discreteStateAction[3] > DISC_PFOV_MAX) ? DISC_PFOV_MAX : discreteStateAction[3];

	return discreteStateAction;
}