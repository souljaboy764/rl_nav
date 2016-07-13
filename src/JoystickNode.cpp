#include <algorithm>
#include <limits>
#include <numeric>
#include <functional>
#include <thread>
#include <chrono>

#include <cmath>
#include <ctime>
#include <cstdlib>

#include "Helper.h"
#include "JoystickNode.h"

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/ModelState.h>

#include <tf/transform_datatypes.h>

#include <turtlebot_nav/ExpectedPath.h>
/*#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/so3.h>
*/

using namespace std;

pthread_mutex_t JoystickNode::pose_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::cpose_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::ptam_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::markerArray_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::odom_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::vel_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::plannerStatus_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::globalPlanner_mutex = PTHREAD_MUTEX_INITIALIZER;

JoystickNode::JoystickNode()
{
	srand (time(NULL));
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1);
	planner_pub = nh.advertise<std_msgs::Float32MultiArray>("/planner/input",1);
	global_planner_pub = nh.advertise<std_msgs::Empty>("/planner/input/global",1);
	planner_reset_pub = nh.advertise<std_msgs::Empty>("/planner/reset",1);
	path_pub = nh.advertise<visualization_msgs::Marker>("/robot_path",1);
	gazebo_state_reset_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1);
	ptam_com_pub = nh.advertise<std_msgs::String>("/vslam/key_pressed",1);
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/my_pose",1);
	next_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/my_next_pose",1);
	expected_pub = nh.advertise<geometry_msgs::PoseStamped>("/expected_pose",1);
	init_pub = nh.advertise<std_msgs::Empty>("/rl/init",1);
	sendCommand_pub = nh.advertise<std_msgs::Empty>("/rl/sendCommand",1);

	//expectedPathClient = nh.serviceClient<turtlebot_nav::ExpectedPath>("/planner/global/expected_path");
	
	state = 0;
	rlRatio = 10;
	breakCount = 0;
	joy.buttons = vector<int>(11,0);
	joy.axes = vector<float>(8,0);
	prevQ = 1;	
	robotName = "";
	just_init = false;
	initialized = false;
	num_inits = 0;
	initY = 0;
	
	qFile.open("qIRLData.txt",ios::app);
	
	path.id=0;
	path.lifetime=ros::Duration(1);
	path.header.frame_id = "/world";
	path.header.stamp = ros::Time::now();
	path.ns = "robotpose_publisher";
	path.action = visualization_msgs::Marker::ADD;
	path.type = visualization_msgs::Marker::LINE_STRIP;
	path.color.r=1.0;
	path.color.g=1.0;
	path.color.a=1.0;
	path.scale.x=0.01;
	path.pose.orientation.w=1.0;
	path.points.clear();

	joy_sub = nh.subscribe("/joy", 100, &JoystickNode::joyCb, this);
	init_sub = nh.subscribe("/rl/init", 100, &JoystickNode::initCb, this);
	sendCommand_sub = nh.subscribe("/rl/sendCommand", 100, &JoystickNode::sendCommandCb, this);
	pose_sub = nh.subscribe("/vslam/pose_world",100, &JoystickNode::poseCb, this);
	cam_pose_sub = nh.subscribe("/vslam/pose",100, &JoystickNode::camPose, this);
	odom_sub = nh.subscribe("/odom", 100, &JoystickNode::odomCb, this);
	pointCloud_sub = nh.subscribe("/vslam/frame_points", 100, &JoystickNode::pointCloudCb, this);
	ptamInfo_sub = nh.subscribe("/vslam/info", 100, &JoystickNode::ptamInfoCb, this);
	plannerStatus_sub = nh.subscribe("/planner/status", 100, &JoystickNode::plannerStatusCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &JoystickNode::gazeboModelStatesCb, this);
	globalPoints_sub = nh.subscribe("/planner/global/path", 100, &JoystickNode::globalPathCb, this);

	ros::NodeHandle p_nh("~");
	p_nh.getParam("mode", MODE);
	p_nh.getParam("num_episodes", NUM_EPISODES);
	p_nh.getParam("max_steps", MAX_STEPS);
	cout<<"MODE: "<<MODE<<endl;
	if(MODE.length()>0)
	{
		state = 1;
		init_pub.publish(std_msgs::Empty());
	}
}

JoystickNode::~JoystickNode()
{
	qFile.close();
	ros::NodeHandle p_nh("~");
	p_nh.deleteParam("mode");
	p_nh.deleteParam("num_episodes");
	p_nh.deleteParam("max_steps");
}

void JoystickNode::initCb(const std_msgs::EmptyPtr emptyPtr)
{
	std_msgs::String resetString, spaceString;
	gazebo_msgs::ModelState newState;
	geometry_msgs::Twist twist;
	initY = 0;
	planner_reset_pub.publish(std_msgs::Empty());//stop planner

	//qFile<<"NEW SESSION AT "<<ros::Time::now()<<'\n';

	resetString.data = "r";
	spaceString.data = "Space";
	ptam_com_pub.publish(resetString);
	ptam_com_pub.publish(resetString);
	ptam_com_pub.publish(resetString);
	ptam_com_pub.publish(resetString);
	
	newState.model_name = robotName;
	newState.reference_frame = "world";

/*	//map 1
	newState.pose.position.x = 2.6;
	newState.pose.position.y = 2.6;
	newState.pose.position.z = 0;
	newState.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, PI/4.0);*/

	//training map
	newState.pose.position.x = -1;
	newState.pose.position.y = 1;
	newState.pose.position.z = 0;
	
	newState.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.4);

	gazebo_state_reset_pub.publish(newState);
	ptam_com_pub.publish(spaceString);

	twist.linear.x=-0.4;
	clock_t t = clock();
	ros::Rate r(10);
	while(((float) (clock() - t))/CLOCKS_PER_SEC < 0.3)	
	{
		vel_pub.publish(twist);
		r.sleep();
	}
	
	num_broken = 0;	
	just_init=true;
	initialized = false;
	ros::Rate(1).sleep();
	ptam_com_pub.publish(spaceString);	
}

void JoystickNode::poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr)
{
	pthread_mutex_lock(&pose_mutex);
	pose = *posePtr;
	pose.header.frame_id = "world";

	vector<double> orientation;
	ros::Rate r(10);
	if(just_init) 
	{
		just_init=false;
		num_inits++;
		orientation = Helper::getPoseOrientation(pose.pose.pose.orientation);
		orientation[0] = abs(orientation[0]);
		cout<<orientation[0]<<" "<<orientation[1]<<" "<<orientation[2]<<" "<<(orientation[0]-3.14)*(orientation[0]-3.14)<<endl;
		if((orientation[0]-3.14)*(orientation[0]-3.14) > 0.003)
			init_pub.publish(std_msgs::Empty());
		else
		{
			initialized = true;
			if(state)
				sendCommand_pub.publish(std_msgs::Empty());
		}
	}
	else
		initialized = true;
	
	if(!initY)
		initY = pose.pose.pose.position.y;
	//float trace = pose.pose.covariance[0] + pose.pose.covariance[7] + pose.pose.covariance[14] + pose.pose.covariance[21] + pose.pose.covariance[28] + pose.pose.covariance[35];
	//if(sqrt(inner_product(pose.pose.covariance.begin(), pose.pose.covariance.end(), pose.pose.covariance.begin(), 0.0)) > 0.03)
	//if(trace > 0.03)
	cout<<initY<<" "<<pose.pose.pose.position.y<<" "<<(initY - pose.pose.pose.position.y)*(initY - pose.pose.pose.position.y) <<endl;
	if((initY - pose.pose.pose.position.y)*(initY - pose.pose.pose.position.y) >=0.15)
		num_broken++;
	else if(num_broken>0)
		num_broken--;
	
	path.points.push_back(pose.pose.pose.position);
	path_pub.publish(path);
	
	geometry_msgs::PoseStamped ps;
	ps.header = pose.header;
	ps.pose = pose.pose.pose;
	pose_pub.publish(ps);
		
	pthread_mutex_unlock(&pose_mutex);
}

void JoystickNode::globalPathCb(const std_msgs::Float32MultiArrayPtr arrayPtr)
{
	pthread_mutex_lock(&globalPlanner_mutex);
	vector<float> input = arrayPtr->data;
	//float Q = learner.getQ(learner.getRLInput(input, pointCloud));
	float Q = get<2>(learner.getAction(input,pointCloud));
	geometry_msgs::PoseStamped ps;
	ps = Helper::getPoseFromInput(input, pose);
	expected_pub.publish(ps);
	if(Q<Q_THRESH)
	{
		cout<<robotWorldPose.position.x<<" "<<robotWorldPose.position.y<<" "<<Helper::getPoseOrientation(robotWorldPose.orientation)[2]<<endl;
		cout<<Q<< " " ;
		cout<<pointCloud.points.size();
		cout<<" "<<input[5]<<endl;
		planner_reset_pub.publish(std_msgs::Empty());//stop planner
		//state  = 0;
		state = 1;
		sendCommand_pub.publish(std_msgs::Empty());
	}
	pthread_mutex_unlock(&globalPlanner_mutex);
}

void JoystickNode::camPose(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr)
{
	pthread_mutex_lock(&cpose_mutex);
	cpose = *posePtr;
	pthread_mutex_unlock(&cpose_mutex);
}

void JoystickNode::joyCb(const sensor_msgs::JoyPtr joyPtr)
{
	geometry_msgs::Twist command;
	if(joyPtr->buttons[POWER] and !joy.buttons[POWER])
		cout << breakCount << " " << rlRatio << " " << num_steps << " " << episodeList.size() <<endl;
/*	if((joyPtr->axes[DH] and !joy.axes[DH]) and joyPtr->axes[DH] != joy.axes[DH])
	{
		if(joyPtr->axes[DH]==1)
		{
			right = not right;
			if(left == false and right == false)
				left = true;
		}
		else if(joyPtr->axes[DH]==-1)
		{
			left = not left;
			if(left == false and right == false)
				right = true;
		}
	}
	else if((joyPtr->axes[DV] and !joy.axes[DV]) and joyPtr->axes[DV] != joy.axes[DV])
	{
		if(joyPtr->axes[DV]==1)
		{
			up = not up;
			if(up == false and down == false)
				down = true;
		}
		else if(joyPtr->axes[DV]==-1)
		{
			down = not down;
			if(up == false and down == false)
				up = true;
		}
	}
*/	else if(joyPtr->buttons[RB] and !joy.buttons[RB])
	{
		std_msgs::String resetString;
		resetString.data = "r";
		ptam_com_pub.publish(resetString);
	}
	else if(joyPtr->buttons[LB] and !joy.buttons[LB])
	{
		std_msgs::String spaceString;
		spaceString.data = "Space";
		ptam_com_pub.publish(spaceString);
	}
	else if(joyPtr->buttons[START] and !joy.buttons[START])
		init_pub.publish(std_msgs::Empty());
	else if(joyPtr->buttons[A] and !state)
	{
		//send input to planner
		state = 1;
		sendCommand_pub.publish(std_msgs::Empty());
	}
	else if(joyPtr->buttons[B]  and !joy.buttons[B])
	{
		if(!state)
			planner_reset_pub.publish(std_msgs::Empty());//stop planner
		state = 0;
	}
	else if(joyPtr->buttons[Y])
	{
		num_broken = 0;
		breakCount = 0;
	}
	else if(joyPtr->buttons[X] and !joy.buttons[X])
	{
		state = 2;
		global_planner_pub.publish(std_msgs::Empty());
	}
	else if((fabs(joyPtr->axes[LV])>0.009 or fabs(joyPtr->axes[RH])>0.009) and !state)
	{
		command.linear.x = joyPtr->axes[LV];
		command.angular.z = joyPtr->axes[RH];
		vel_pub.publish(command);
	}
	joy = *joyPtr;
}


void JoystickNode::odomCb(const nav_msgs::OdometryPtr odomPtr)
{	
	pthread_mutex_lock(&odom_mutex);
	odomMsgs.push_back(*odomPtr);
	pthread_mutex_unlock(&odom_mutex);
}

void JoystickNode::pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr)	
{
	pthread_mutex_lock(&pointCloud_mutex);
	pointCloud = *pointCloudPtr;
	pthread_mutex_unlock(&pointCloud_mutex);
}

void JoystickNode::ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr)	
{
	pthread_mutex_lock(&ptam_mutex);
	ptamInfo = *ptamInfoPtr;
	pthread_mutex_unlock(&ptam_mutex);
}

void JoystickNode::gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr)
{
	pthread_mutex_lock(&gazeboModelState_mutex);	
	robotWorldPose = modelStatesPtr->pose.back();
	robotName = modelStatesPtr->name.back();
	pthread_mutex_unlock(&gazeboModelState_mutex);
}

void JoystickNode::plannerStatusCb(const std_msgs::StringPtr plannerStatusPtr)
{
	pthread_mutex_lock(&plannerStatus_mutex);
	if(!(plannerStatusPtr->data.compare("DONE")))
	{
		breakCount++;
		ptam_com::ptam_info info;
		
		pthread_mutex_lock(&ptam_mutex);
		info = ptamInfo;
		pthread_mutex_unlock(&ptam_mutex);
		lastRLInput[3] = (num_broken < 3 and info.trackingQuality)?2:0;
		vector<unsigned int> discreteStateAction = learner.discretizeState(lastRLInput);
		for(auto i: lastRLInput)
			qFile<<i<<'\t';
		for(auto i: discreteStateAction)
			qFile<<i<<'\t';
		qFile << pointCloud.points.size() << '\t' << prevQ << "\t" << ((info.trackingQuality)?2:0) << '\n';
		episode.push_back(lastRLInput);
		//learner.updateQ(lastRLInput, next.second);

		if(num_broken>3 or !info.trackingQuality) 
		{	
			cout<<"Breaking after "<<breakCount<< " steps due to action with Q value "<< prevQ<<'\t';
			for(auto i: lastRLInput)
				cout<<i<<'\t'; 
			cout<< endl;
			breakCount = 0;
			initialized = false;
			
			episodeList.push_back(episode);
			episode.clear();

			if(episodeList.size()==NUM_EPISODES or num_steps >= MAX_STEPS)
			{
				if(!MODE.compare("TRAIN"))
				{
					learner.episodeUpdate(episodeList);
					episodeList.clear();
					rlRatio+=10;
					num_steps = 0;
					cout<<"rlRatio: "<<rlRatio<<endl;
					if(rlRatio==90)
						//ros::shutdown();	
					{
						cout<<"SWITCHING TO TESTING PHASE"<<endl;
						MODE = "TEST";
					}
				}
				else if(!MODE.compare("TEST"))
				{
					Helper::saveFeatureExpectation(episodeList);
					ros::shutdown();
				}
			}
			init_pub.publish(std_msgs::Empty());
		}

		else if(state)
			sendCommand_pub.publish(std_msgs::Empty());
		/*
		if(state and breakCount==1)
		{
			state = 2;
			breakCount = 0;
			global_planner_pub.publish(std_msgs::Empty());
		}
		else if(state)
		{
		/*	turtlebot_nav::ExpectedPath expectedPath;
			expectedPathClient.call(expectedPath);
			vector<float> poses = expectedPath.response.expectedPath.data;
			vector<float> first(poses.begin(),poses.begin()+12), second(poses.begin()+12,poses.begin()+24), secondIP(poses.begin()+24,poses.end());
			pcl::PointCloud<pcl::PointXYZ> firstPC = getPointCloud(first), secondPC = getPointCloud(second), currentPC = pointCloud;
			vector<pcl::PointXYZ> firstCommon = Helper::pointCloudIntersection(currentPC, firstPC), secondCommon = Helper::pointCloudIntersection(firstPC, secondPC);
			float Q1 = learner.getQ(getRLInput(first,currentPC,firstPC)), Q2 = learner.getQ(getRLInput(secondIP,firstPC,secondPC));
			cout<<"Q1: "<<Q1<<" Q2: "<<Q2<<endl;
			if(Q1>Q_THRESH and Q2>Q_THRESH)
				global_planner_pub.publish(std_msgs::Empty());
			sendCommand_pub.publish(std_msgs::Empty());
		}
		else
			planner_reset_pub.publish(std_msgs::Empty());//stop planner*/
	}
	pthread_mutex_unlock(&plannerStatus_mutex);

}

//---------------------- Sending commands to the robot -----------------------------------

void JoystickNode::sendCommandCb(std_msgs::EmptyPtr emptyPtr)
{

	if(initialized)
	{
		std_msgs::Float32MultiArray planner_input;
		vector<vector<float> > inputs = Helper::getTrajectories();
		try 
		{
			pthread_mutex_lock(&pointCloud_mutex);
			prevPointCloud = pointCloud;
			pthread_mutex_unlock(&pointCloud_mutex);

			//Best Q without repeated forward backward actions
			//tie(lastCommand, lastRLInput, prevQ) = learner.getBestQStateAction(lastCommand, prevPointCloud);
			
				
			/*
			//epsilon greedy
			if((rand() % 100) < rlRatio)
				tie(lastCommand, lastRLInput, prevQ) = learner.getBestQStateAction(lastCommand, prevPointCloud);
			else
				tie(lastCommand, lastRLInput, prevQ) = learner.getRandomStateAction(prevPointCloud);
			*/

			
			//incremental training epsilon greedy
			if(!MODE.compare("TRAIN"))
			{
				if((rand() % 100) < rlRatio)
					tie(lastCommand, lastRLInput, prevQ) = learner.getBestQStateAction(lastCommand, prevPointCloud);
				else
					tie(lastCommand, lastRLInput, prevQ) = learner.getRandomStateAction(prevPointCloud);
			}
			else if(!MODE.compare("TEST"))
				tie(lastCommand, lastRLInput, prevQ) = learner.getBestQStateAction(lastCommand, prevPointCloud);
			else
				tie(lastCommand, lastRLInput, prevQ) = learner.getRandomStateAction(prevPointCloud);
			num_steps++;
	/*
			//Thresholded random aligning with global path
			vector<vector<float> > possibleTrajs, possibleInp;
			float q;
			for(vector<vector<float> >::iterator it = inputs.begin(); it!= inputs.end() ; it++)
			{
				vector<float> inp = getRLInput(*it, prevPointCloud);
				q = learner.getQ(inp);
				if(q>Q_THRESH)
				{
					possibleTrajs.push_back(*it);
					possibleInp.push_back(inp);
				}
			}
			if(possibleTrajs.size()<1)
			{
				tie(lastCommand, lastRLInput, prevQ) = learner.getBestQStateAction(lastCommand, prevPointCloud);
			}
			else
			{
				int index = rand()%possibleTrajs.size();
				vector<float> myPose = {robotWorldPose.position.x,robotWorldPose.position.y,tan(Helper::getPoseOrientation(robotWorldPose.orientation)[2])}, nextPose;
				vector<vector<float> > points = {{4.0,7.0,0.0},
												 {6.0,2.0,-tan(PI/4)}};
				if(myPose[0]>=4)
					nextPose = points[1];
				else
					nextPose = points[0];
				float min_dist = numeric_limits<float>::infinity(),ang_dist;
				for(vector<vector<float> >::iterator it = possibleTrajs.begin(); it!= possibleTrajs.end() ; it++)
				{	
					ang_dist = fabs(atan(nextPose[2]) - (atan(myPose[2]) + (*it)[12]*atan((*it)[5])));
					if(ang_dist<=min_dist)
					{
						min_dist = ang_dist;
						index = it - possibleTrajs.begin();
					}
				}
				lastCommand = possibleTrajs[index];
				lastRLInput = possibleInp[index];
				prevQ = learner.getQ(possibleInp[index]);
			}
	*/

/*			//Thresholded random
			int i = 20;
			do 
			{
				tie(lastCommand, lastRLInput, prevQ) = learner.getRandomStateAction(prevPointCloud);
				i--;
			}while(prevQ<Q_THRESH and i>=0);
*/

/*
			//Purely random
				tie(lastCommand, lastRLInput, prevQ) = learner.getRandomStateAction(prevPointCloud);
*/			
			next_pose_pub.publish(Helper::getPoseFromInput(lastCommand, pose));

			planner_input.data = lastCommand;
			path.points.clear();
			odomMsgs.clear();

			planner_pub.publish(planner_input);//send input to planner
		}
		catch (int e)
		{
			cout<<"Exception "<<e<<endl;
			state = 0;
		}
	}
	
}
