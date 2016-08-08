#include <algorithm>
#include <limits>
#include <numeric>
#include <functional>
#include <thread>

#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstdio>

#include "Helper.h"
#include "JoystickNode.h"

#include <tf/transform_datatypes.h>

#include <turtlebot_nav/ExpectedPath.h>

using namespace std;

pthread_mutex_t JoystickNode::pose_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::ptamInfo_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::plannerStatus_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::globalPlanner_mutex = PTHREAD_MUTEX_INITIALIZER;

JoystickNode::JoystickNode()
{
	srand (time(NULL));
	vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
	planner_pub = nh.advertise<std_msgs::Float32MultiArray>("/planner/input",1);
	global_planner_pub = nh.advertise<std_msgs::Empty>("/planner/input/global",1);
	planner_reset_pub = nh.advertise<std_msgs::Empty>("/planner/reset",1);
	gazebo_state_reset_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1);
	ptam_com_pub = nh.advertise<std_msgs::String>("/vslam/key_pressed",1);
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/my_pose",1);
	next_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/my_next_pose",1);
	expected_pub = nh.advertise<geometry_msgs::PoseStamped>("/expected_pose",1);
	init_pub = nh.advertise<std_msgs::Empty>("/rl/init",1);
	sendCommand_pub = nh.advertise<std_msgs::Empty>("/rl/sendCommand",1);

	expectedPathClient = nh.serviceClient<turtlebot_nav::ExpectedPath>("/planner/global/expected_path");
	
	ifstream ratioFile("ratioFile.txt");
	ratioFile >> rlRatio;
	ratioFile.close();
	if(!rlRatio)
		rlRatio = 10;
	episodeList = Helper::readFeatureExpectation("tempfeFile.txt");
	state = 0;
	breakCount = 0;
	joy.buttons = vector<int>(11,0);
	joy.axes = vector<float>(8,0);
	prevQ = 1;	
	initState.model_name = "";
	just_init = false;
	initialized = false;
	initY = 0;
	num_broken = 0;
	Q_THRESH = 0;
		
	qFile.open("qIRLData_SVM.txt",ios::app);
	
	joy_sub = nh.subscribe("/joy", 100, &JoystickNode::joyCb, this);
	init_sub = nh.subscribe("/rl/init", 100, &JoystickNode::initCb, this);
	sendCommand_sub = nh.subscribe("/rl/sendCommand", 100, &JoystickNode::sendCommandCb, this);
	pose_sub = nh.subscribe("/vslam/pose_world",100, &JoystickNode::poseCb, this); 
	pointCloud_sub = nh.subscribe("/vslam/frame_points", 100, &JoystickNode::pointCloudCb, this);
	ptamInfo_sub = nh.subscribe("/vslam/info", 100, &JoystickNode::ptamInfoCb, this);
	ptamStart_sub = nh.subscribe("/vslam/started", 100, &JoystickNode::ptamStartedCb, this);
	plannerStatus_sub = nh.subscribe("/planner/status", 100, &JoystickNode::plannerStatusCb, this);
	globalPoints_sub = nh.subscribe("/planner/global/path", 100, &JoystickNode::globalNextPoseCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &JoystickNode::gazeboModelStatesCb, this);


	initState.reference_frame = "world";
	initState.pose.position.z = 0;

	float initYaw;
	MAP=-1;
	ros::NodeHandle p_nh("~");
	p_nh.getParam("qThresh", Q_THRESH);
	p_nh.getParam("mode", MODE);
	p_nh.getParam("num_episodes", MAX_EPISODES);
	p_nh.getParam("max_steps", MAX_STEPS);
	p_nh.getParam("init_x", initState.pose.position.x);
	p_nh.getParam("init_y", initState.pose.position.y);
	p_nh.getParam("init_Y", initYaw);
	p_nh.getParam("map", MAP);
	
	initState.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, initYaw);

	if(!MODE.compare("TRAIN") or !MODE.compare("TEST"))
	{
		state = 1;
		init_pub.publish(std_msgs::Empty());
	}
	else if(!MODE.compare("MAP"))
		state = 2;
}

JoystickNode::~JoystickNode()
{
	qFile.close();
	//cout<<"Quitting JoystickNode"<<endl;
	ros::NodeHandle p_nh("~");
	p_nh.deleteParam("qThresh");
	p_nh.deleteParam("mode");
	p_nh.deleteParam("num_episodes");
	p_nh.deleteParam("max_steps");
	p_nh.deleteParam("init_x");
	p_nh.deleteParam("init_y");
	p_nh.deleteParam("init_Y");
	p_nh.deleteParam("map");

	remove("tempfeFile.txt");
	Helper::saveFeatureExpectation(episodeList, "tempfeFile.txt");

	ofstream ratioFile("ratioFile.txt");
	ratioFile << ((rlRatio==90)?10:rlRatio) << endl;
	ratioFile.close();
}

void JoystickNode::ptamStartedCb(const std_msgs::EmptyPtr emptyPtr)
{
	episode.clear();
	init_pub.publish(std_msgs::Empty());
}

/**
 *	PTAM Initializer callback
 */
void JoystickNode::initCb(const std_msgs::EmptyPtr emptyPtr)
{
	std_msgs::String resetString, spaceString;
	geometry_msgs::Twist twist;
	initY = 0;
	planner_reset_pub.publish(std_msgs::Empty());//stop planner

	resetString.data = "r";
	spaceString.data = "Space";
	ptam_com_pub.publish(resetString);
	ptam_com_pub.publish(resetString);
	ptam_com_pub.publish(resetString);
	ptam_com_pub.publish(resetString);
	
	gazebo_state_reset_pub.publish(initState);
	ptam_com_pub.publish(spaceString);

	twist.linear.x=-0.4;
	clock_t t = clock();
	ros::Rate r(10);
	while(((float) (clock() - t))/CLOCKS_PER_SEC < 0.2)	
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

/**
 *	Receive PTAM pose of camera in world
 */
void JoystickNode::poseCb(const geometry_msgs::PoseWithCovarianceStampedPtr posePtr)
{
	pthread_mutex_lock(&pose_mutex);
	pose = *posePtr;
	pose.header.frame_id = "world";
	
	vector<double> orientation;
	double angle;
	if(just_init) 
	{
		just_init=false;
		orientation = Helper::getPoseOrientation(pose.pose.pose.orientation);
		angle = abs(orientation[0]);
		////cout<<orientation[0]<<" "<<orientation[1]<<" "<<orientation[2]<<" "<<(orientation[0]-3.14)*(orientation[0]-3.14)<<endl;
		//if((orientation[0]-3.14)*(orientation[0]-3.14) > 0.003)
		if((angle-3.14)*(angle-3.14) > 0.003)
			init_pub.publish(std_msgs::Empty());
		else
		{
			initialized = true;
			if(state==1)
				sendCommand_pub.publish(std_msgs::Empty());
			else if(state==2)
				global_planner_pub.publish(std_msgs::Empty());
		}
	}
	else
		initialized = true;

	if(!initY)
		initY = pose.pose.pose.position.y;
	//float trace = pose.pose.covariance[0] + pose.pose.covariance[7] + pose.pose.covariance[14] + pose.pose.covariance[21] + pose.pose.covariance[28] + pose.pose.covariance[35];
	//if(sqrt(inner_product(pose.pose.covariance.begin(), pose.pose.covariance.end(), pose.pose.covariance.begin(), 0.0)) > 0.03)
	//if(trace > 0.03)
	if((initY - pose.pose.pose.position.y)*(initY - pose.pose.pose.position.y) >=0.15)
		num_broken++;
	else if(num_broken>0)
		num_broken--;
	
	geometry_msgs::PoseStamped ps;
	ps.header = pose.header;
	ps.pose = pose.pose.pose;
	pose_pub.publish(ps);
	
	pthread_mutex_unlock(&pose_mutex);
}

/**
 *	Receive next expected robot pose w.r.t. current pose along global path
 */
void JoystickNode::globalNextPoseCb(const std_msgs::Float32MultiArrayPtr arrayPtr)
{
	pthread_mutex_lock(&globalPlanner_mutex);
	vector<float> input = arrayPtr->data;
	
	float Q = get<2>(learner.getAction(input));
	geometry_msgs::PoseStamped ps;
	ps = Helper::getPoseFromInput(input, pose);
	expected_pub.publish(ps);
	ptam_com::ptam_info info;
		
	pthread_mutex_lock(&ptamInfo_mutex);
	info = ptamInfo;
	pthread_mutex_unlock(&ptamInfo_mutex);
	
	if(num_broken>3 or !info.trackingQuality)
		planner_reset_pub.publish(std_msgs::Empty());//stop planner
	else if(!MODE.compare("MAP") and Q<Q_THRESH)
	{
		//cout<<robotWorldPose.position.x<<" "<<robotWorldPose.position.y<<" "<<Helper::getPoseOrientation(robotWorldPose.orientation)[2]<<endl;
		//cout<<Q<< " " ;
		//cout<<pointCloud.points.size();
		//cout<<" "<<input[5]<<endl;
		planner_reset_pub.publish(std_msgs::Empty());//stop planner
		//state  = 0;
		vector<float> goal;
		if(MAP==1)
			goal = {6,2};
		else if(MAP==2)
			goal = {7,6};
		state = 1;
		if((robotWorldPose.position.x - goal[0])*(robotWorldPose.position.x - goal[0]) + (robotWorldPose.position.y - goal[1])*(robotWorldPose.position.y - goal[1]) > 0.25)
			sendCommand_pub.publish(std_msgs::Empty());
		else //goal reached
			planner_reset_pub.publish(std_msgs::Empty()); 
	}

	
	pthread_mutex_unlock(&globalPlanner_mutex);
}

/**
 *	Receive joystick input
 */
void JoystickNode::joyCb(const sensor_msgs::JoyPtr joyPtr)
{
	geometry_msgs::Twist command;
	if(joyPtr->buttons[POWER] and !joy.buttons[POWER])
		cout << breakCount << " " << rlRatio << " " << num_steps << " " << num_episodes <<endl;
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
	else if(joyPtr->buttons[BACK] and !joy.buttons[BACK])
		ros::shutdown();
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

/**
 *	Receive PTAM point cloud
 */
void JoystickNode::pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr)	
{
	pthread_mutex_lock(&pointCloud_mutex);
	pointCloud = *pointCloudPtr;
	pthread_mutex_unlock(&pointCloud_mutex);
}

/**
 *	Receive PTAM Info
 */
void JoystickNode::ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr)	
{
	pthread_mutex_lock(&ptamInfo_mutex);
	ptamInfo = *ptamInfoPtr;
	pthread_mutex_unlock(&ptamInfo_mutex);
}

/**
 *	Receive Gazebo Models data
 */
void JoystickNode::gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr)
{
	pthread_mutex_lock(&gazeboModelState_mutex);	
	robotWorldPose = modelStatesPtr->pose.back();
	if(just_init and not initState.model_name.length() and modelStatesPtr->name.back().length())
		init_pub.publish(std_msgs::Empty());
	initState.model_name = modelStatesPtr->name.back();
	pthread_mutex_unlock(&gazeboModelState_mutex);
}

/**
 *	Receive Planner status after executing local action
 */
void JoystickNode::plannerStatusCb(const std_msgs::StringPtr plannerStatusPtr)
{
	pthread_mutex_lock(&plannerStatus_mutex);
	
	if(!(plannerStatusPtr->data.compare("DONE")))
	{
		breakCount++;
		ptam_com::ptam_info info;
		
		pthread_mutex_lock(&ptamInfo_mutex);
		info = ptamInfo;
		pthread_mutex_unlock(&ptamInfo_mutex);
		if(!episode.size() or (episode.size() and lastRLInput!=episode.back()))
		{
			lastRLInput.push_back((num_broken <= 3 and info.trackingQuality)?0:1);
			for(auto i: lastRLInput)
				qFile<<i<<'\t';
			/*qFile<<';';
			for(auto input : Helper::getTrajectories())
			{
				vector<int> rlInput;
				float Q;
				tie(ignore, rlInput, Q) = learner.getAction(input);
				for(auto i : rlInput)
					qFile<<i<<'\t';
				qFile<<';';
			}*/
			qFile << '\n';
			episode.push_back(lastRLInput);
		}

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
			num_episodes++;

			if(MODE.compare("MAP") and num_episodes == MAX_EPISODES)// or num_steps >= MAX_STEPS)
			{
				if(!MODE.compare("TRAIN"))
				{
					learner.episodeUpdate(episodeList);
					episodeList.clear();
					rlRatio+=10;
					num_steps = 0;
					//cout<<"rlRatio: "<<rlRatio<<endl;
					if(rlRatio==90)
						//ros::shutdown();	
					{
						//cout<<"SWITCHING TO TESTING PHASE"<<endl;
						MODE = "TEST";
					}
				}
				else if(!MODE.compare("TEST"))
				{
					Helper::saveFeatureExpectation(episodeList, "feFile.txt");
					remove("tempfeFile.txt");
					ros::shutdown();
				}
			}
			if(!MODE.compare("MAP"))
				planner_reset_pub.publish(std_msgs::Empty());//stop planner
			else
				init_pub.publish(std_msgs::Empty());
		
		}
		else if(state==1)
		{
			if(!MODE.compare("MAP"))
			{
				state = 2;
				breakCount = 0;
				global_planner_pub.publish(std_msgs::Empty());
			}
			else
				sendCommand_pub.publish(std_msgs::Empty());
			
		}
		else if(state==2)
		{
			turtlebot_nav::ExpectedPath expectedPath;
			expectedPathClient.call(expectedPath);
			vector<float> poses = expectedPath.response.expectedPath.data;
			vector<float> first(poses.begin(),poses.begin()+12), second(poses.begin()+12,poses.begin()+24), secondIP(poses.begin()+24,poses.end());
			pcl::PointCloud<pcl::PointXYZ> firstPC = Helper::getPointCloudAtPosition(first), secondPC = Helper::getPointCloudAtPosition(second), currentPC = pointCloud;
			//vector<pcl::PointXYZ> firstCommon = Helper::pointCloudIntersection(currentPC, firstPC), secondCommon = Helper::pointCloudIntersection(firstPC, secondPC);
			float Q1, Q2;
			tie(ignore,ignore,Q1) = learner.getAction(first);//,currentPC));
			tie(ignore,ignore,Q2) = learner.getAction(secondIP);//,firstPC));
			//float Q2 = learner.getQ(Helper::getRLInput(secondIP,firstPC));
			cout<<"Q1: "<<Q1<<" Q2: "<<Q2<<endl;
			if(Q1>Q_THRESH and Q2>Q_THRESH)
				global_planner_pub.publish(std_msgs::Empty());
			else
				sendCommand_pub.publish(std_msgs::Empty());
		}
		else
			planner_reset_pub.publish(std_msgs::Empty());//stop planner*/
	}
	
	pthread_mutex_unlock(&plannerStatus_mutex);

}

/**
 *	Sending commands to the robot 
 */
void JoystickNode::sendCommandCb(std_msgs::EmptyPtr emptyPtr)
{
	if(initialized)
	{
		std_msgs::Float32MultiArray planner_input;
		vector<vector<float> > inputs = Helper::getTrajectories();
		
		//incremental training epsilon greedy
		if(!MODE.compare("TRAIN"))
			tie(lastCommand, lastRLInput, prevQ) = learner.getEpsilonGreedyStateAction(rlRatio,lastCommand);
		else if(!MODE.compare("TEST"))
			tie(lastCommand, lastRLInput, prevQ) = learner.getEpsilonGreedyStateAction(95,lastCommand);
		else if(!MODE.compare("MAP"))
		{
			vector<float> myPose = {robotWorldPose.position.x,robotWorldPose.position.y};
			float nextAngle;
			vector<vector<float> > points;
			if(MAP==1)//map 1
				points = {{4.0,7.0,0.0}, {6.0,2.0,PI/4}};
			else if(MAP==2)//map 2
				points = {{4.0,4.0,PI/4}, {7.0,6.0,0.0}};
					
			if(myPose[0]>=4)
				nextAngle = points[1][2];
			else
				nextAngle = points[0][2];
					
			tie(lastCommand, lastRLInput, prevQ) = learner.getThresholdedClosestAngleStateAction(Q_THRESH, nextAngle, lastCommand);
	
		}	
		
		else
			tie(lastCommand, lastRLInput, prevQ) = learner.getSLRandomStateAction();

		
				num_steps++;
		next_pose_pub.publish(Helper::getPoseFromInput(lastCommand, pose));
		
		planner_input.data = lastCommand;

		planner_pub.publish(planner_input);//send input to planner
	}	
}
