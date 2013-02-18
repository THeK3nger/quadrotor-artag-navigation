/*
*  Gazebo - Outdoor Multi-Robot Simulator
*  Copyright (C) 2003
*     Nate Koenig & Andrew Howard
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*/
/*
* Desc: Ros Laser controller.
* Author: Nathan Koenig
* Date: 01 Feb 2007
* SVN info: $Id: gazebo_ros_laser.cpp 6683 2008-06-25 19:12:30Z natepak $
*/

#include <algorithm>
#include <assert.h>

#include <labrob_quadrotor_sensors/labrob_sonar_controller.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/RaySensor.hh>

namespace gazebo{

GZ_REGISTER_DYNAMIC_CONTROLLER("labrob_sonar_controller", labrobLaserController);

////////////////////////////////////////////////////////////////////////////////
// Constructor
labrobLaserController::labrobLaserController(Entity *parent): Controller(parent){
	myParent = dynamic_cast<RaySensor*>(parent);
	
	if (!myParent)
		gzthrow("labrobLaserController controller requires a Ray Sensor as its parent");
	
	Param::Begin(&parameters);
		robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
		hokuyoMinIntensityP = new ParamT<double>("hokuyoMinIntensity", 101.0, 0);
		gaussianNoiseP = new ParamT<double>("gaussianNoise", 0.0, 0);
		topicNameP = new ParamT<std::string>("topicName", "", 1);
		deprecatedTopicNameP = new ParamT<std::string>("deprecatedTopicName", "", 0);
		frameNameP = new ParamT<std::string>("frameName", "default_gazebo_ros_laser_frame", 0);
		numParentP = new ParamT<int>("numParent", 0, 0);
	Param::End();
	
	laserConnectCount = 0;
	deprecatedLaserConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
labrobLaserController::~labrobLaserController(){
	delete robotNamespaceP;
	delete hokuyoMinIntensityP;
	delete gaussianNoiseP;
	delete topicNameP;
	delete deprecatedTopicNameP;
	delete frameNameP;
	delete rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void labrobLaserController::LoadChild(XMLConfigNode *node){
	robotNamespaceP->Load(node);
	robotNamespace = robotNamespaceP->GetValue();
	
	if (!ros::isInitialized()){
		int argc = 0;
		char** argv = NULL;
		ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
	}
	
	rosnode_ = new ros::NodeHandle(robotNamespace);
	
	hokuyoMinIntensityP->Load(node);
	hokuyoMinIntensity = hokuyoMinIntensityP->GetValue();
	ROS_INFO("INFO: gazebo_ros_laser plugin artifically sets minimum intensity to %f due to cutoff in hokuyo filters." , hokuyoMinIntensity);
	
	topicNameP->Load(node);
	
	// Changing name
// 	topicName = parent->GetParent()->GetParent()->GetName();
// 	topicName += "/";
// 	topicName += topicNameP->GetValue();
	
	numParentP->Load(node);
	std::string topicToSub;
	if(robotNamespace=="/"){
		gazebo::Entity* actParent;
		actParent = parent;
		for(int i=0;i<numParentP->GetValue();++i)
			actParent = actParent->GetParent();
		
		std::cout << "actParent->GetName(): " << actParent->GetName() << std::endl;
		robotNamespace = actParent->GetName();
		topicToSub = robotNamespace + "/";
	}
	
	deprecatedTopicNameP->Load(node);
	deprecatedTopicName = deprecatedTopicNameP->GetValue();
	frameNameP->Load(node);
	frameName = frameNameP->GetValue();
	gaussianNoiseP->Load(node);
	gaussianNoise = gaussianNoiseP->GetValue();
	
	if (topicNameP->GetValue() != ""){ 
		topicToSub += topicNameP->GetValue();
		topicName = topicNameP->GetValue();
#ifdef USE_CBQ
		ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
			topicToSub,1,
			boost::bind( &labrobLaserController::LaserConnect,this),
			boost::bind( &labrobLaserController::LaserDisconnect,this), ros::VoidPtr(), &laser_queue_);
		pub_ = rosnode_->advertise(ao);
#else
		pub_ = rosnode_->advertise<sensor_msgs::LaserScan>(topicToSub,1,
			boost::bind( &labrobLaserController::LaserConnect, this),
			boost::bind( &labrobLaserController::LaserDisconnect, this));
#endif
	}

	if (deprecatedTopicName != "")
	{
#ifdef USE_CBQ
		ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
			deprecatedTopicName,1,
			boost::bind( &labrobLaserController::DeprecatedLaserConnect,this),
			boost::bind( &labrobLaserController::DeprecatedLaserDisconnect,this), ros::VoidPtr(), &laser_queue_);
		deprecated_pub_ = rosnode_->advertise(ao);
#else
		deprecated_pub_ = rosnode_->advertise<sensor_msgs::LaserScan>(deprecatedTopicName,1,
			boost::bind( &labrobLaserController::DeprecatedLaserConnect, this),
			boost::bind( &labrobLaserController::DeprecatedLaserDisconnect, this));
#endif
	}
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void labrobLaserController::InitChild()
{
	// sensor generation off by default
	myParent->SetActive(false);
#ifdef USE_CBQ
	// start custom queue for laser
	callback_queue_thread_ = boost::thread( boost::bind( &labrobLaserController::LaserQueueThread,this ) );
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void labrobLaserController::LaserConnect()
{
	laserConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void labrobLaserController::LaserDisconnect()
{
	laserConnectCount--;

	if (laserConnectCount == 0 && deprecatedLaserConnectCount == 0)
		myParent->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void labrobLaserController::DeprecatedLaserConnect()
{
	ROS_WARN("you are subscribing to a deprecated ROS topic %s, please change your code/launch script to use new ROS topic %s",
					deprecatedTopicName.c_str(), topicName.c_str());
	deprecatedLaserConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void labrobLaserController::DeprecatedLaserDisconnect()
{
	deprecatedLaserConnectCount--;

	if (laserConnectCount == 0 && deprecatedLaserConnectCount == 0)
		myParent->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void labrobLaserController::UpdateChild()
{
	// as long as ros is connected, parent is active
	//ROS_ERROR("debug laser count %d",laserConnectCount);
	if (!myParent->IsActive())
	{
		// do this first so there's chance for sensor to run 1 frame after activate
		if ((laserConnectCount > 0 && topicName != "") ||
				(deprecatedLaserConnectCount > 0 && deprecatedTopicName != ""))
			myParent->SetActive(true);
	}
	else
	{
		PutLaserData();
	}
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void labrobLaserController::FiniChild()
{
	laser_queue_.clear();
	laser_queue_.disable();
	rosnode_->shutdown();
	sleep(1);
#ifdef USE_CBQ
	callback_queue_thread_.join();
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void labrobLaserController::PutLaserData()
{
	int i, ja, jb;
	double ra, rb, r, b;
	double intensity;

	Angle maxAngle = myParent->GetMaxAngle();
	Angle minAngle = myParent->GetMinAngle();

	double maxRange = myParent->GetMaxRange();
	double minRange = myParent->GetMinRange();
	int rayCount = myParent->GetRayCount();
	int rangeCount = myParent->GetRangeCount();

	/***************************************************************/
	/*                                                             */
	/*  point scan from laser                                      */
	/*                                                             */
	/***************************************************************/
	lock.lock();
	// Add Frame Name
	laserMsg.header.frame_id = frameName;
	laserMsg.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
	laserMsg.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;


	double tmp_res_angle = (maxAngle.GetAsRadian() - minAngle.GetAsRadian())/((double)(rangeCount -1)); // for computing yaw
	laserMsg.angle_min = minAngle.GetAsRadian();
	laserMsg.angle_max = maxAngle.GetAsRadian();
	laserMsg.angle_increment = tmp_res_angle;
	laserMsg.time_increment  = 0; // instantaneous simulator scan
	laserMsg.scan_time       = 0; // FIXME: what's this?
	laserMsg.range_min = minRange;
	laserMsg.range_max = maxRange;
	laserMsg.ranges.clear();
	laserMsg.intensities.clear();

	// Interpolate the range readings from the rays
	for (i = 0; i<rangeCount; i++)
	{
		b = (double) i * (rayCount - 1) / (rangeCount - 1);
		ja = (int) floor(b);
		jb = std::min(ja + 1, rayCount - 1);
		b = b - floor(b);

		assert(ja >= 0 && ja < rayCount);
		assert(jb >= 0 && jb < rayCount);

		ra = std::min(myParent->GetRange(ja) , maxRange-minRange); // length of ray
		rb = std::min(myParent->GetRange(jb) , maxRange-minRange); // length of ray

		// Range is linear interpolation if values are close,
		// and min if they are very different
		//if (fabs(ra - rb) < 0.10)
			r = (1 - b) * ra + b * rb;
		//else r = std::min(ra, rb);

		// Intensity is averaged
		intensity = 0.5*( myParent->GetRetro(ja) + (int) myParent->GetRetro(jb));

		/***************************************************************/
		/*                                                             */
		/*  point scan from laser                                      */
		/*                                                             */
		/***************************************************************/
		laserMsg.ranges.push_back(std::min(r + minRange + GaussianKernel(0,gaussianNoise), maxRange));
		laserMsg.intensities.push_back(std::max(hokuyoMinIntensity,intensity + GaussianKernel(0,gaussianNoise)));
	}

	// send data out via ros message
	if (laserConnectCount > 0 && topicName != "")
			pub_.publish(laserMsg);

	if (deprecatedLaserConnectCount > 0 && deprecatedTopicName != "")
			deprecated_pub_.publish(laserMsg);

	lock.unlock();

}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double labrobLaserController::GaussianKernel(double mu,double sigma)
{
	// using Box-Muller transform to generate two independent standard normally disbributed normal variables
	// see wikipedia
	double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
	double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
	double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
	//double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
	// we'll just use X
	// scale to our mu and sigma
	X = sigma * X + mu;
	return X;
}

#ifdef USE_CBQ
////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void labrobLaserController::LaserQueueThread()
{
	static const double timeout = 0.01;

	while (rosnode_->ok())
	{
		laser_queue_.callAvailable(ros::WallDuration(timeout));
	}
}
#endif

}
