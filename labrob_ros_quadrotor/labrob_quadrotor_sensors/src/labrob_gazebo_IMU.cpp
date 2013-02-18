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
* Desc: 3D position interface for ground truth.
* Author: Sachin Chitta and John Hsu
* Date: 1 June 2008
* SVN info: $Id$
*/

#include <labrob_quadrotor_sensors/labrob_gazebo_IMU.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("labrob_gazebo_IMU", labrob_gazebo_IMU);


////////////////////////////////////////////////////
///////////////////// Constructor //////////////////
////////////////////////////////////////////////////
labrob_gazebo_IMU::labrob_gazebo_IMU(Entity *parent ) : Controller(parent){
	myParent = dynamic_cast<Model*>(parent);
	
	std::cout << "Parent: " << myParent->GetName() << std::endl;
	for(int i=0;i<myParent->GetChildren().size();++i)
		std::cout << "Children["<< i << "]: " << myParent->GetChildren().at(i)->GetName() << std::endl;
	
	if (!myParent)
			gzthrow("labrob_gazebo_IMU controller requires a Model as its parent");
	
	Param::Begin(&parameters);
		robotNamespaceP				= new ParamT<std::string>("robotNamespace", "/", 0);
		bodyNameP							= new ParamT<std::string>("bodyName", "", 0);
		frameNameP 						= new ParamT<std::string>("frameName", "", 0); // deprecated, warning if specified by user
		topicNameP 						= new ParamT<std::string>("topicName", "", 1);
		topicNameVelP 				= new ParamT<std::string>("topicNameVel", "", 1);
		deprecatedTopicNameP 	= new ParamT<std::string>("deprecatedTopicName", "", 0);
		xyzOffsetsP						= new ParamT<Vector3>("xyzOffsets", Vector3(0,0,0),0);
		rpyOffsetsP						= new ParamT<Vector3>("rpyOffsets", Vector3(0,0,0),0);
		gaussianNoiseP				= new ParamT<double>("gaussianNoise",0.0,0);
		serviceNameP					= new ParamT<std::string>("serviceName","torso_lift_imu/calibrate", 0);
		numParentP						= new ParamT<int>("numParent",0, 0);
	Param::End();
	
	imuConnectCount = 0;
	deprecatedImuConnectCount = 0;
}

////////////////////////////////////////////////////
///////////////////// Destructor ///////////////////
////////////////////////////////////////////////////
labrob_gazebo_IMU::~labrob_gazebo_IMU(){
	delete robotNamespaceP;
	delete bodyNameP;
	delete frameNameP;
	delete topicNameP;
	delete deprecatedTopicNameP;
	delete xyzOffsetsP;
	delete rpyOffsetsP;
	delete gaussianNoiseP;
	delete serviceNameP;
	delete rosnode_;
	delete numParentP;
	delete topicNameVelP;
}

////////////////////////////////////////////////////
/////////////////// Load the controller ////////////
////////////////////////////////////////////////////
void labrob_gazebo_IMU::LoadChild(XMLConfigNode *node){
	robotNamespaceP->Load(node);
	robotNamespace = robotNamespaceP->GetValue();
	if (!ros::isInitialized()){
		int argc = 0;
		char** argv = NULL;
		ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
	}
	
	rosnode_ = new ros::NodeHandle(robotNamespace);
	
	bodyNameP->Load(node);
	bodyName = bodyNameP->GetValue();
	std::cout << "bodyName: " << bodyName << std::endl;
	
	// assert that the body by bodyName exists
	if (dynamic_cast<Body*>(myParent->GetBody(bodyName)) == NULL)
		ROS_FATAL("labrob_gazebo_IMU plugin error: bodyName: %s does not exist\n",bodyName.c_str());
	
	myBody = dynamic_cast<Body*>(myParent->GetBody(bodyName));
	
	frameNameP->Load(node);
	if (frameNameP->GetValue() != "")
		ROS_WARN("Deprecating the ability to specify imu frame_id, this now defaults to the parent imu_link name.  Angular velocity and linear acceleration is in local frame and orientation starts with the transform from gazebo world frame to imu_link frame.  This is done to mimick hardware on PR2.");
	
	numParentP->Load(node);
	std::string topicToSub;
	std::string topicToSubVel;
	if(robotNamespace=="/"){
		gazebo::Entity* actParent;
		actParent = parent;
		for(int i=0;i<numParentP->GetValue();++i)
			actParent = actParent->GetParent();
		
		std::cout << "actParent->GetName()IMU: " << actParent->GetName() << std::endl;
		robotNamespace = actParent->GetName();
		topicToSub = robotNamespace + "/";
		topicToSubVel = robotNamespace + "/";
	}
	
	topicNameP->Load(node);
	topicName = topicNameP->GetValue();
	deprecatedTopicNameP->Load(node);
	deprecatedTopicName = deprecatedTopicNameP->GetValue();
	
	xyzOffsetsP->Load(node);
	xyzOffsets = xyzOffsetsP->GetValue();
	rpyOffsetsP->Load(node);
	rpyOffsets = rpyOffsetsP->GetValue();
	gaussianNoiseP->Load(node);
	gaussianNoise = gaussianNoiseP->GetValue();
	
	if (topicName != ""){
		topicToSub += topicNameP->GetValue();
		topicName = topicNameP->GetValue();
		#ifdef USE_CBQ
				ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::Imu>(
					topicToSub,1,
					boost::bind( &labrob_gazebo_IMU::IMUConnect,this),
					boost::bind( &labrob_gazebo_IMU::IMUDisconnect,this), ros::VoidPtr(), &imu_queue_);
				pub_ = rosnode_->advertise(ao);
		#else
				pub_ = rosnode_->advertise<sensor_msgs::Imu>(topicName,10);
		#endif
	}
	
	topicNameVelP->Load(node);
	topicNameVel = topicNameVelP->GetValue();
	if (topicNameVel != ""){
		topicToSubVel += topicNameVelP->GetValue();
		topicNameVel = topicNameVelP->GetValue();
		#ifdef USE_CBQ
				ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<geometry_msgs::Vector3Stamped>(
					topicToSubVel,1,
					boost::bind( &labrob_gazebo_IMU::IMUConnectVel,this),
					boost::bind( &labrob_gazebo_IMU::IMUDisconnectVel,this), ros::VoidPtr(), &imu_queue_);
				pubVel_ = rosnode_->advertise(ao);
		#else
				pubVel_ = rosnode_->advertise<geometry_msgs::Vector3Stamped>(topicNameVel,10);
		#endif
	}
	
	if (deprecatedTopicName != ""){
		#ifdef USE_CBQ
				ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::Imu>(
					deprecatedTopicName,1,
					boost::bind( &labrob_gazebo_IMU::DeprecatedIMUConnect,this),
					boost::bind( &labrob_gazebo_IMU::DeprecatedIMUDisconnect,this), ros::VoidPtr(), &imu_queue_);
				deprecated_pub_ = rosnode_->advertise(ao);
		#else
				deprecated_pub_ = rosnode_->advertise<sensor_msgs::Imu>(deprecatedTopicName,10);
		#endif
	}
	
	// add service call version for position change
	serviceNameP->Load(node);
	serviceName = serviceNameP->GetValue();
	// advertise services on the custom queue
	ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
			serviceName,boost::bind( &labrob_gazebo_IMU::ServiceCallback, this, _1, _2 ), ros::VoidPtr(), &imu_queue_);
	srv_ = rosnode_->advertiseService(aso);
	
}

/////////////////////////////////////////////////////////////////////
//////////// returns true always, imu is always calibrated in sim ///
/////////////////////////////////////////////////////////////////////
bool labrob_gazebo_IMU::ServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	return true;
}

////////////////////////////////////////////////////
////////////////// Increment count /////////////////
////////////////////////////////////////////////////
void labrob_gazebo_IMU::IMUConnect(){
	imuConnectCount++;
}

////////////////////////////////////////////////////
/////////////// Decrement count ////////////////////
////////////////////////////////////////////////////
void labrob_gazebo_IMU::IMUDisconnect(){
	imuConnectCount--;
}

////////////////////////////////////////////////////
////////////////// Increment count /////////////////
////////////////////////////////////////////////////
void labrob_gazebo_IMU::IMUConnectVel(){
	imuConnectCountVel++;
}

////////////////////////////////////////////////////
/////////////// Decrement count ////////////////////
////////////////////////////////////////////////////
void labrob_gazebo_IMU::IMUDisconnectVel(){
	imuConnectCountVel--;
}

////////////////////////////////////////////////////
/////////////////// Increment count ////////////////
////////////////////////////////////////////////////
void labrob_gazebo_IMU::DeprecatedIMUConnect(){
	ROS_WARN("you are subscribing to a deprecated ROS topic %s, please change your code/launch script to use new ROS topic %s",
					deprecatedTopicName.c_str(), topicName.c_str());
	deprecatedImuConnectCount++;
}

////////////////////////////////////////////////////
//////////////// Decrement count ///////////////////
////////////////////////////////////////////////////
void labrob_gazebo_IMU::DeprecatedIMUDisconnect(){
	deprecatedImuConnectCount--;
}

////////////////////////////////////////////////////
///////////// Initialize the controller ////////////
////////////////////////////////////////////////////
void labrob_gazebo_IMU::InitChild(){
	last_time = Simulator::Instance()->GetSimTime();
	//initial_pose = myBody->GetPose(); // get initial pose of the local link
	last_vpos = myBody->GetWorldLinearVel(); // get velocity in gazebo frame
	last_veul = myBody->GetWorldAngularVel(); // get velocity in gazebo frame
	apos = 0;
	aeul = 0;
	#ifdef USE_CBQ
		// start custom queue for imu
		callback_queue_thread_ = boost::thread( boost::bind( &labrob_gazebo_IMU::IMUQueueThread,this ) );
	#endif
}

////////////////////////////////////////////////////
/////////////// Update the controller //////////////
////////////////////////////////////////////////////
void labrob_gazebo_IMU::UpdateChild(){
	
	if ((imuConnectCount > 0 && topicName != "") || (deprecatedImuConnectCount > 0 && deprecatedTopicName != "")){
		Pose3d pose;
		Quatern rot;
		Vector3 pos;

		// Get Pose/Orientation ///@todo: verify correctness
		pose = myBody->GetWorldPose(); // - myBody->GetCoMPose();
		// apply xyz offsets and get position and rotation components
		pos = pose.pos + xyzOffsets;
		rot = pose.rot;
		// std::cout << " --------- labrob_gazebo_IMU rot " << rot.x << ", " << rot.y << ", " << rot.z << ", " << rot.u << std::endl;
		
		// apply rpy offsets
		Quatern qOffsets;
		qOffsets.SetFromEuler(rpyOffsets);
		rot = qOffsets*rot;
		rot.Normalize();
		
		gazebo::Time cur_time = Simulator::Instance()->GetSimTime();
		
		// get Rates
		Vector3 vpos = myBody->GetWorldLinearVel(); // get velocity in gazebo frame
		Vector3 veul = myBody->GetWorldAngularVel(); // get velocity in gazebo frame
		
		// differentiate to get accelerations
		double tmp_dt = last_time.Double() - cur_time.Double();
		if (tmp_dt != 0){
			apos = (last_vpos - vpos) / tmp_dt;
			aeul = (last_veul - veul) / tmp_dt;
			last_vpos = vpos;
			last_veul = veul;
		}
		
		lock.lock();
		
		
		// copy data into pose message
		imuMsg.header.frame_id = bodyName;
		imuMsg.header.stamp.sec = cur_time.sec;
		imuMsg.header.stamp.nsec = cur_time.nsec;
		
		// orientation quaternion
		
		// uncomment this if we are reporting orientation in the local frame
		// not the case for our imu definition
		// // apply fixed orientation offsets of initial pose
		// rot = initial_pose.rot*rot;
		// rot.Normalize();
		
		imuMsg.orientation.x = rot.x;
		imuMsg.orientation.y = rot.y;
		imuMsg.orientation.z = rot.z;
		imuMsg.orientation.w = rot.u;
		
		// pass euler angular rates
		Vector3 linear_velocity(veul.x + GaussianKernel(0,gaussianNoise)
													,veul.y + GaussianKernel(0,gaussianNoise)
													,veul.z + GaussianKernel(0,gaussianNoise));
		// rotate into local frame
		// @todo: deal with offsets!
		linear_velocity = rot.RotateVector(linear_velocity);
		imuMsg.angular_velocity.x    = linear_velocity.x;
		imuMsg.angular_velocity.y    = linear_velocity.y;
		imuMsg.angular_velocity.z    = linear_velocity.z;
		
		// pass accelerations
		Vector3 linear_acceleration(apos.x + GaussianKernel(0,gaussianNoise)
															,apos.y + GaussianKernel(0,gaussianNoise)
															,apos.z + GaussianKernel(0,gaussianNoise));
		// rotate into local frame
		// @todo: deal with offsets!
		linear_acceleration = rot.RotateVector(linear_acceleration);
		imuMsg.linear_acceleration.x    = linear_acceleration.x;
		imuMsg.linear_acceleration.y    = linear_acceleration.y;
		imuMsg.linear_acceleration.z    = linear_acceleration.z;
		
		// fill in covariance matrix
		/// @todo: let user set separate linear and angular covariance values.
		/// @todo: apply appropriate rotations from frame_pose
		imuMsg.orientation_covariance[0] = gaussianNoise*gaussianNoise;
		imuMsg.orientation_covariance[4] = gaussianNoise*gaussianNoise;
		imuMsg.orientation_covariance[8] = gaussianNoise*gaussianNoise;
		imuMsg.angular_velocity_covariance[0] = gaussianNoise*gaussianNoise;
		imuMsg.angular_velocity_covariance[4] = gaussianNoise*gaussianNoise;
		imuMsg.angular_velocity_covariance[8] = gaussianNoise*gaussianNoise;
		imuMsg.linear_acceleration_covariance[0] = gaussianNoise*gaussianNoise;
		imuMsg.linear_acceleration_covariance[4] = gaussianNoise*gaussianNoise;
		imuMsg.linear_acceleration_covariance[8] = gaussianNoise*gaussianNoise;
		
		// publish to ros
		if (imuConnectCount > 0 && topicName != "")
			pub_.publish(imuMsg);
		
		if (deprecatedImuConnectCount > 0 && deprecatedTopicName != "")
			deprecated_pub_.publish(imuMsg);
		
		lock.unlock();
		
		// save last time stamp
		last_time = cur_time;
	}
	
	// If requested, publish velocity data
	if (imuConnectCountVel > 0 && topicNameVel != ""){
		gazebo::Time cur_time = Simulator::Instance()->GetSimTime();
		geometry_msgs::Vector3Stamped velBody;
		Vector3 velBodyTmp = myBody->GetRelativeLinearVel();
		velBody.header.frame_id = bodyName;
		velBody.header.stamp.sec = cur_time.sec;
		velBody.header.stamp.nsec = cur_time.nsec;
		velBody.vector.x = velBodyTmp.x;
		velBody.vector.y = velBodyTmp.y;
		velBody.vector.z = velBodyTmp.z;
		pubVel_.publish(velBody);
	}
}

////////////////////////////////////////////////////
///////////// Finalize the controller //////////////
////////////////////////////////////////////////////
void labrob_gazebo_IMU::FiniChild(){
	rosnode_->shutdown();
	callback_queue_thread_.join();
}

////////////////////////////////////////////////////
////////////// Utility for adding noise ////////////
////////////////////////////////////////////////////
double labrob_gazebo_IMU::GaussianKernel(double mu,double sigma){
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
////////////////////////////////////////////////////
//////////// Put laser data to the interface ///////
////////////////////////////////////////////////////
void labrob_gazebo_IMU::IMUQueueThread(){
	static const double timeout = 0.01;
	
	while (rosnode_->ok()){
		imu_queue_.callAvailable(ros::WallDuration(timeout));
	}
}
#endif