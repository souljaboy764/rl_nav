#include <iostream>
#include <fstream>
#include "Helper.h"

using namespace std;

pthread_mutex_t Helper::pose_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;

geometry_msgs::PoseWithCovarianceStamped Helper::pose;
ptam_com::ptam_info Helper::ptamInfo;
geometry_msgs::Pose Helper::robotWorldPose;
pcl::PointCloud<pcl::PointXYZ> Helper::currentPointCloud;
ros::ServiceClient Helper::posePointCloudClient;

Helper::Helper()
{
	posePointCloudClient = nh.serviceClient<ptam_com::PosePointCloud>("/vslam/posepointcloud");
	pose_sub = nh.subscribe("/vslam/pose_world",100, &Helper::poseCb, this);
	info_sub = nh.subscribe("/vslam/info",100, &Helper::ptamInfoCb, this);
	pointCloud_sub = nh.subscribe("/vslam/frame_points", 100, &Helper::pointCloudCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &Helper::gazeboModelStatesCb, this);
}

void Helper::poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr)
{
	pthread_mutex_lock(&pose_mutex);
	pose = *posePtr;
	pthread_mutex_unlock(&pose_mutex);
}

void Helper::ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr)
{
	pthread_mutex_lock(&info_mutex);
	ptamInfo = *ptamInfoPtr;
	pthread_mutex_unlock(&info_mutex);
}

void Helper::gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr)
{
	pthread_mutex_lock(&gazeboModelState_mutex);	
	robotWorldPose = modelStatesPtr->pose.back();
	pthread_mutex_unlock(&gazeboModelState_mutex);
}

void Helper::pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr)	
{
	pthread_mutex_lock(&pointCloud_mutex);
	currentPointCloud = *pointCloudPtr;
	pthread_mutex_unlock(&pointCloud_mutex);
}

pcl::PointCloud<pcl::PointXYZ> Helper::getPointCloudAtPosition(vector<float> input)
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


vector<pcl::PointXYZ> Helper::pointCloudIntersection(pcl::PointCloud<pcl::PointXYZ> pointCloudA, pcl::PointCloud<pcl::PointXYZ> pointCloudB)
{
	vector<pcl::PointXYZ> commonPoints(pointCloudB.width * pointCloudB.height + pointCloudA.width * pointCloudA.height);
	vector<pcl::PointXYZ>::iterator it;
	it=set_intersection(pointCloudB.points.begin(), pointCloudB.points.end(), 
						pointCloudA.points.begin(), pointCloudA.points.end(), commonPoints.begin(),pointEqComparer());
	commonPoints.resize(it-commonPoints.begin());
	return commonPoints;
}

bool Helper::inLimits(float x, float y)
{
	//return x>0.4 and y > 0.4 and x < 7.6 and y < 7.6 and (x<3.6 or x>4.4 or (x>=3.6 and x<=4.4 and y>6.4)); // map 1
	return x>=-6 and x<=0 and y>=-1 and y<=3; //training map
	//return true;
}

vector<vector<float> > Helper::getTrajectories()
{
	float angle = PI/90.0, num_angles = 14;
	vector<vector<float> > inputs;
	vector<double> orientation = getPoseOrientation(robotWorldPose.orientation);
	
	for(float i=-num_angles*angle ; i<=num_angles*angle ; i+=angle)
	{			
		vector<float> inp = {0.0,0.0,0.0, 
							 cos(i), sin(i), tan(i),
							 0.0,0.0,0.0,0.0,0.0,0.0,
							 1.0,0.0,1.5};
		float x = robotWorldPose.position.x + cos(orientation[2] + i);
		float y = robotWorldPose.position.y + sin(orientation[2] + i);
		if(inLimits(x,y))
			inputs.push_back(inp);

		inp[3] *= -1.0;
		inp[5] *= -1.0;
		inp[12] *= -1.0;
		x = robotWorldPose.position.x - cos(orientation[2] - i);
		y = robotWorldPose.position.y - sin(orientation[2] - i);
		if(inLimits(x,y))
			inputs.push_back(inp);
	}
	return inputs;
}

void Helper::saveFeatureExpectation(vector<vector<vector<unsigned int> > > episodeList, string fileName)
{
	ofstream feFile(fileName);
	for(vector<vector<vector<unsigned int> > >::iterator episode = episodeList.begin(); episode!=episodeList.end(); ++episode)
		for(vector<vector<unsigned int> >::iterator rlStep = episode->begin(); rlStep!=episode->end(); ++rlStep)
		{
			for(vector<unsigned int>::iterator it=rlStep->begin(); it!=rlStep->end()-1; ++it)
				feFile<< *it << '\t';
			feFile << rlStep->back()<<endl;
		}
}

vector<vector<vector<unsigned int> > > Helper::readFeatureExpectation(string fileName)
{
	vector<vector<vector<unsigned int> > > episodeList;
	vector<vector<unsigned int> > episode;
	ifstream infile(fileName);
	if(infile.good())
	{
		unsigned int dir, angle, fov, status;
		while(infile)
		{
			infile >> dir >> angle >> fov >> status;
			episode.push_back({dir, angle, fov, status});

			if(!status)
			{
				if(!episodeList.size() or (episodeList.size() and episode.back() != episodeList.back().back()))
					episodeList.push_back(episode);
				episode.clear();
			}
		}
	}

	return episodeList;
}