#include "SarsaLearner.h"
#include <iostream>
#include <fstream>

int main()
{
	SarsaLearner learner;
	vector<vector<vector<float> > > episodeList;
	vector<vector<float> > episode;
	ifstream inFile("infile");
	while(!inFile.eof())
	{
		vector<float> input(4);
		for(int i=0;i<4;i++)
		{
			inFile>>input[i];
			cout<<input[i]<<" ";
		}
		cout<<endl;

		episode.push_back(input);
		if(input[3]==0)
		{
			episodeList.push_back(episode);
			episode.clear();
		}
	}
	learner.episodeUpdate(episodeList);
	return 0;
}