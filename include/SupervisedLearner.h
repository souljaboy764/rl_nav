#include <vector>

using namespace std;

class SupervisedLearner
{
protected:

	bool slValid;
	// Matrix dimension sizes
	static const int STATE_DIR_MAX = 2;
	static const int STATE_HEAD_MAX = 20;
	static const int STATE_FOV_MAX = 20;

	bool slPredictionMatrix[STATE_DIR_MAX][STATE_HEAD_MAX][STATE_FOV_MAX]; // Matrix for sl predictions
	float slValueMatrix[STATE_DIR_MAX][STATE_HEAD_MAX][STATE_FOV_MAX]; // Matrix for sl distance

public:
	SupervisedLearner();
	bool predict(vector<int> stateAction);
	float distance(vector<int> stateAction);
};