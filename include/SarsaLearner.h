#include <vector>

using namespace std;


class SarsaLearner
{
private:

	bool qValid;
	// Q Matrix dimension sizes
	static const int STATE_DIR_MAX = 2;
	static const int STATE_HEAD_MAX = 30;//60;
	static const int STATE_FOV_MAX = 20;
//	static const int STATE_PFOV_MAX = 20;

	// Limits of dimensions
	static const int MAX_DIR = 2;
	static const int MIN_DIR = 1;
	static const int MAX_HEAD = 30;
	static const int MIN_HEAD = 0;//-30;
	static const int MAX_FOV = 30;
	static const int MIN_FOV = 0;
//	static const int MAX_PFOV = 100;
//	static const int MIN_PFOV = 0;

	// Dimension discretization factors
	static const int DISC_DIR = 1;
	static const int DISC_HEAD = 1;
	static constexpr float DISC_FOV = 1.5f;
	static const int DISC_PFOV = 5;

	// Dimension discretization limits
	static const int DISC_DIR_MAX = 1;
	static const int DISC_HEAD_MAX = 29;//59;
	static const int DISC_FOV_MAX = 19;
//	static const int DISC_PFOV_MAX = 19;

	// Dimension offsets
	static const int OFFSET_DIR = -1;
	static const int OFFSET_HEAD = 0;//30;
	static const int OFFSET_FOV = 0;
//	static const int OFFSET_PFOV = 0;

	// SARSA parameters
	static constexpr float SARSA_ALPHA = 0.5f;
	static constexpr float SARSA_GAMMA = 0.9f;

	// Reward function features
	static const int NUM_FEATURES_SA = 5;

protected:
	float qMatrix[STATE_DIR_MAX][STATE_HEAD_MAX][STATE_FOV_MAX]; // Matrix for Q Values
	vector<float> w; //reward weights


public:
	SarsaLearner();
	~SarsaLearner();
	float getQ(vector<float> stateAction);
	void updateQ(vector<float> stateAction, vector<float> nextStateAction);
	void updateQ(vector<float> stateAction, float qNext);
	void episodeUpdate(vector<vector<vector<float> > > episodeList);
	vector<unsigned int> discretizeState(vector<float> stateAction);
	float getReward(vector<float> stateAction);
	/*struct QInput
	{
		vector<float> rlInput;
		double nextQ;
		bool broken;
	};*/

};