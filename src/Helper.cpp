#include <iostream>
#include <fstream>
#include "Helper.h"

using namespace std;

pthread_mutex_t Helper::pose_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;

geometry_msgs::PoseWithCovarianceStamped Helper::pose;
ptam_com::ptam_info Helper::ptamInfo;
geometry_msgs::Pose Helper::robotWorldPose;

ros::ServiceClient Helper::posePointCloudClient;

Helper::Helper()
{
	posePointCloudClient = nh.serviceClient<ptam_com::PosePointCloud>("/vslam/posepointcloud");
	pose_sub = nh.subscribe("/vslam/pose_world",100, &Helper::poseCb, this);
	info_sub = nh.subscribe("/vslam/info",100, &Helper::ptamInfoCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &Helper::gazeboModelStatesCb, this);
}

void Helper::poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr)
{
	pthread_mutex_lock(&pose_mutex);
	Helper::pose = *posePtr;
	pthread_mutex_unlock(&pose_mutex);
}

void Helper::ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr)
{
	pthread_mutex_lock(&info_mutex);
	Helper::ptamInfo = *ptamInfoPtr;
	pthread_mutex_unlock(&info_mutex);
}

void Helper::gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr)
{
	pthread_mutex_lock(&gazeboModelState_mutex);	
	robotWorldPose = modelStatesPtr->pose.back();
	pthread_mutex_unlock(&gazeboModelState_mutex);
}

pcl::PointCloud<pcl::PointXYZ> Helper::getPointCloud(vector<float> input)
{
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	ptam_com::PosePointCloud posePointCloud;
	
	//PoseStamped from the new point
	pthread_mutex_lock(&pose_mutex);
	posePointCloud.request.pose = getPoseFromInput(input, Helper::pose);
	pthread_mutex_unlock(&pose_mutex);

	posePointCloudClient.call(posePointCloud);
	pcl::fromROSMsg(posePointCloud.response.pointCloud, pointCloud);
	
	return pointCloud;
}

vector<float> Helper::getRLInput(vector<float> input, pcl::PointCloud<pcl::PointXYZ> currentPointCloud)
{
	//Calculate intersection of current and next pointcloud
	vector<float> rl_input;
	pcl::PointCloud<pcl::PointXYZ> nextPointCloud = getPointCloud(input);
	
	vector<pcl::PointXYZ> commonPoints = pointCloudIntersection(currentPointCloud,nextPointCloud);
	float dir = input[12], del_heading = atan(input[5]);
	//RL params
	rl_input.push_back((dir==1)?2.0:1.0);
	if(fabs(del_heading)*180.0/PI > 30)
		rl_input.push_back(30);
	else
		rl_input.push_back(fabs((del_heading)*180.0/PI));

	rl_input.push_back(min(30,commonPoints.size()/20.0));

	pthread_mutex_lock(&info_mutex);
	rl_input.push_back((Helper::ptamInfo.trackingQuality)?2:0);
	pthread_mutex_unlock(&info_mutex);
//	rl_input.push_back(100.0*commonPoints.size()/float(currentPointCloud.points.size()));
	//rl_input.push_back(distance);

	return rl_input;

}

vector<double> Helper::getPoseOrientation(geometry_msgs::Quaternion quat)
{
	double roll, pitch, yaw;
	tf::Quaternion q;
	tf::quaternionMsgToTF(quat, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	return {roll, pitch, yaw};
}

geometry_msgs::PoseStamped Helper::getPoseFromInput(vector<float> input, geometry_msgs::PoseWithCovarianceStamped pose)
{
	geometry_msgs::PoseStamped p_out;
	geometry_msgs::Pose currentPose, p, newPose;
	tf::Quaternion currentQuat;
	tf::Pose currentTfPose, newTfPose;

	currentPose = pose.pose.pose;
	
	p.position.z = input[3];
	p.position.x = -input[4]*input[12];
	p.position.y = 0.0;
	p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -input[12]*atan(input[5]),0.0);
	tf::quaternionMsgToTF(currentPose.orientation, currentQuat);
	tf::Transform currentTF(tf::Matrix3x3(currentQuat), tf::Vector3(currentPose.position.x,currentPose.position.y,currentPose.position.z));
	
	tf::poseMsgToTF(p, currentTfPose);
	newTfPose = currentTF * currentTfPose;
	tf::poseTFToMsg(newTfPose, newPose);
	
	p_out.header = pose.header;
	p_out.header.frame_id = "world";
	p_out.pose = newPose;
	return p_out;
}


vector<pcl::PointXYZ> Helper::pointCloudIntersection(pcl::PointCloud<pcl::PointXYZ> currentPointCloud, pcl::PointCloud<pcl::PointXYZ> nextPointCloud)
{
	vector<pcl::PointXYZ> commonPoints(nextPointCloud.width * nextPointCloud.height + currentPointCloud.width * currentPointCloud.height);
	vector<pcl::PointXYZ>::iterator it;
	it=set_intersection(nextPointCloud.points.begin(), nextPointCloud.points.end(), 
						currentPointCloud.points.begin(), currentPointCloud.points.end(), commonPoints.begin(),pointEqComparer());
	commonPoints.resize(it-commonPoints.begin());
	return commonPoints;
}

bool Helper::inLimits(float x, float y)
{
	//return x>0.3 and y > 0.3 and x < 7.7 and y < 7.7 and (x<3.7 or x>4.3 or (x>=3.7 and x<=4.3 and y>6.3)); // map 1
	return x>=-6 and x<=0 and y>=-1 and y<=3; //training map
	//return true;
}

vector<vector<float> > Helper::getTrajectories()
{
	float angle = PI/180.0, num = 27, max_num = 55;
	vector<vector<float> > inputs;
	vector<double> angles = getPoseOrientation(robotWorldPose.orientation);
/*	vector<float> inp1 = {0.0,0.0,0.0, 
							 cos(float(i-3)*angle), sin(float(i-3)*angle), tan(float(i-3)*angle),
							 0.0,0.0,0.0,0.0,0.0,0.0,
							 1.0,0.0,1.5};
	inputs.push_back(inp1);*/
	//Generate 14 new positions
	pair<int, int> startEnd =make_pair(0,max_num);
	for(int i=startEnd.first;i<startEnd.second;i++)
	{			
		vector<float> inp = {0.0,0.0,0.0, 
							 cos(float(i-num)*angle), sin(float(i-num)*angle), tan(float(i-num)*angle),
							 0.0,0.0,0.0,0.0,0.0,0.0,
							 1.0,0.0,1.5};
		float x = robotWorldPose.position.x + cos(angles[2] + float(i-num)*angle);
		float y = robotWorldPose.position.y + sin(angles[2] + float(i-num)*angle);
		if(inLimits(x,y))
			inputs.push_back(inp);

		inp[3] *= -1.0;
		inp[5] *= -1.0;
		inp[12] *= -1.0;
		x = robotWorldPose.position.x - cos(angles[2] - float(i-num)*angle);
		y = robotWorldPose.position.y - sin(angles[2] - float(i-num)*angle);
		if(inLimits(x,y))
			inputs.push_back(inp);
	}
	return inputs;
}

geometry_msgs::Pose Helper::getRobotWorldPose()
{
	return robotWorldPose;
}

void Helper::saveFeatureExpectation(vector<vector<vector<float> > > episodeList)
{
	ofstream feFile("feFile.txt");
	for(vector<vector<vector<float> > >::iterator episode = episodeList.begin(); episode!=episodeList.end(); ++episode)
		for(vector<vector<float> >::iterator rlStep = episode->begin(); rlStep!=episode->end(); ++rlStep)
		{
			for(vector<float>::iterator it=rlStep->begin(); it!=rlStep->end()-1; ++it)
				feFile<< *it << '\t';
			feFile << rlStep->back()<<endl;
		}
}