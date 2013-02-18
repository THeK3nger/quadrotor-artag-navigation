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
@mainpage
	Desc: labrob_gazebo_camera_utils plugin for simulating camera_s in Gazebo
	Author: Marco Cognetti
	Date: 22 October 2012
	SVN info: $Id$
@htmlinclude manifest.html
@b labrob_gazebo_camera_utils plugin broadcasts ROS Image messages
*/

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <labrob_quadrotor_sensors/labrob_gazebo_camera_utilsFuerte.h>

#include "physics/World.hh"
#include "physics/HingeJoint.hh"
#include "sensors/Sensor.hh"
#include "sdf/interface/SDF.hh"
#include "sdf/interface/Param.hh"
#include "common/Exception.hh"
#include "sensors/CameraSensor.hh"
#include "sensors/SensorTypes.hh"
#include "rendering/Camera.hh"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"
#include "image_transport/image_transport.h"

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

#include "tf/tf.h"

namespace gazebo{

///////////////////////////////////////////////////////
///////////////////// Constructor /////////////////////
///////////////////////////////////////////////////////
labrob_gazebo_camera_utils::labrob_gazebo_camera_utils(){
	image_connect_count_   = 0;
	info_connect_count_    = 0;
	
	// maintain for one more release for backwards compatibility with pr2_gazebo_plugins
	imageConnectCount      = image_connect_count_;
	infoConnectCount       = info_connect_count_;
	
	last_update_time_      = common::Time(0);
	last_info_update_time_ = common::Time(0);
}

void labrob_gazebo_camera_utils::configCallback(gazebo_plugins::GazeboRosCameraConfig &config, uint32_t level){
	ROS_INFO("Reconfigure request for the gazebo ros camera_: %s. New rate: %.2f", camera_name_.c_str(), config.imager_rate);
	parentSensor_->SetUpdateRate(config.imager_rate);
}

///////////////////////////////////////////////////////
///////////////////// Destructor //////////////////////
///////////////////////////////////////////////////////
labrob_gazebo_camera_utils::~labrob_gazebo_camera_utils(){
	parentSensor_->SetActive(false);
	rosnode_->shutdown();
	camera_queue_.clear();
	camera_queue_.disable();
	callback_queue_thread_.join();
	delete rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void labrob_gazebo_camera_utils::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf){
	// Get the world name.
	std::string world_name = _parent->GetWorldName();
	
	// Get the world_
	world_ = physics::get_world(world_name);
	
	// maintain for one more release for backwards compatibility with pr2_gazebo_plugins
	world = world_;
	
	robot_namespace_ = "";
	if (_sdf->HasElement("robotNamespace"))
		robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
	
	if (_sdf->HasElement("setNamespace")){
		std::string setNm = _sdf->GetElement("setNamespace")->GetValueString();
		std::string pn = _parent->GetParentName();
		if(setNm == "1" || setNm == "true"){
			std::size_t pos;
			pos = pn.find(":");
// 			std::cout << "pos: " << pos << " pnSize: " << std::string::npos << std::endl;
			if(pos!=std::string::npos){
				pn = pn.substr(0,pos);
				robot_namespace_ = pn + "/";
				std::cout << "robotNamespace camera: " << robot_namespace_ << std::endl;
			}else{
				ROS_WARN("A namespace can not include :! Check if the actual namespace cointans it!");
				robot_namespace_ = pn;
			}
		}
	}
	
	image_topic_name_ = "image_raw";
	if (_sdf->GetElement("imageTopicName"))
		image_topic_name_ = _sdf->GetElement("imageTopicName")->GetValueString();
	
	camera_info_topic_name_ = "camera_info";
	if (_sdf->HasElement("cameraInfoTopicName"))
		camera_info_topic_name_ = _sdf->GetElement("cameraInfoTopicName")->GetValueString();
	
	if (!_sdf->HasElement("cameraName"))
		ROS_INFO("Camera plugin missing <cameraName>, default to empty");
	else
		camera_name_ = _sdf->GetElement("cameraName")->GetValueString();
	
	if (!_sdf->HasElement("frameName"))
		ROS_INFO("Camera plugin missing <frameName>, defaults to /world");
	else
		frame_name_ = _sdf->GetElement("frameName")->GetValueString();
	
	if (!_sdf->GetElement("updateRate")){
		ROS_INFO("Camera plugin missing <updateRate>, defaults to 0");
		update_rate_ = 0;
	}else
		update_rate_ = _sdf->GetElement("updateRate")->GetValueDouble();
	
	if (!_sdf->GetElement("CxPrime")){
		ROS_INFO("Camera plugin missing <CxPrime>, defaults to 0");
		cx_prime_ = 0;
	}else
		cx_prime_ = _sdf->GetElement("CxPrime")->GetValueDouble();
	
	if (!_sdf->HasElement("Cx")){
		ROS_INFO("Camera plugin missing <Cx>, defaults to 0");
		cx_= 0;
	}else
		cx_ = _sdf->GetElement("Cx")->GetValueDouble();
	
	if (!_sdf->HasElement("Cy")){
		ROS_INFO("Camera plugin missing <Cy>, defaults to 0");
		cy_= 0;
	}else
		cy_ = _sdf->GetElement("Cy")->GetValueDouble();
	
	if (!_sdf->HasElement("focalLength")){
		ROS_INFO("Camera plugin missing <focalLength>, defaults to 0");
		focal_length_= 0;
	}else
		focal_length_ = _sdf->GetElement("focalLength")->GetValueDouble();
	
	if (!_sdf->HasElement("hackBaseline")){
		ROS_INFO("Camera plugin missing <hackBaseline>, defaults to 0");
		hack_baseline_= 0;
	}else
		hack_baseline_ = _sdf->GetElement("hackBaseline")->GetValueDouble();

	if (!_sdf->HasElement("distortionK1")){
		ROS_INFO("Camera plugin missing <distortionK1>, defaults to 0");
		distortion_k1_= 0;
	}else
		distortion_k1_ = _sdf->GetElement("distortionK1")->GetValueDouble();
	
	if (!_sdf->HasElement("distortionK2")){
		ROS_INFO("Camera plugin missing <distortionK2>, defaults to 0");
		distortion_k2_= 0;
	}else
		distortion_k2_ = _sdf->GetElement("distortionK2")->GetValueDouble();
	
	if (!_sdf->HasElement("distortionK3")){
		ROS_INFO("Camera plugin missing <distortionK3>, defaults to 0");
		distortion_k3_= 0;
	}else
		distortion_k3_ = _sdf->GetElement("distortionK3")->GetValueDouble();
	
	if (!_sdf->HasElement("distortionT1")){
		ROS_INFO("Camera plugin missing <distortionT1>, defaults to 0");
		distortion_t1_= 0;
	}else
		distortion_t1_ = _sdf->GetElement("distortionT1")->GetValueDouble();
	
	if (!_sdf->HasElement("distortionT2")){
		ROS_INFO("Camera plugin missing <distortionT2>, defaults to 0");
		distortion_t2_= 0;
	}else
		distortion_t2_ = _sdf->GetElement("distortionT2")->GetValueDouble();
	
	if ((distortion_k1_ != 0.0) || (distortion_k2_ != 0.0) ||
			(distortion_k3_ != 0.0) || (distortion_t1_ != 0.0) ||
			(distortion_t2_ != 0.0)){
		ROS_WARN("gazebo_ros_camera_ simulation does not support non-zero distortion parameters right now, your simulation maybe wrong.");
	}
	
	
	// Setup ROS
	if (!ros::isInitialized()){
		int argc = 0;
		char** argv = NULL;
		ros::init( argc, argv, "gazebo",
							ros::init_options::NoSigintHandler |
							ros::init_options::AnonymousName );
	}
	
	rosnode_ = new ros::NodeHandle(robot_namespace_+"/"+camera_name_);
	
	itnode_ = new image_transport::ImageTransport(*rosnode_);
	
	// resolve tf prefix
	std::string prefix;
	rosnode_->getParam(std::string("tf_prefix"), prefix);
	frame_name_ = tf::resolve(prefix, frame_name_);
	
	if (!camera_name_.empty()){
		dyn_srv_ = new dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>(*rosnode_);
		dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>::CallbackType f = boost::bind(&labrob_gazebo_camera_utils::configCallback, this, _1, _2);
		dyn_srv_->setCallback(f);
	}
	else{
		ROS_WARN("dynamic reconfigure is not enabled for this image topic [%s] becuase <cameraName> is not specified",image_topic_name_.c_str());
	}
	
	image_pub_ = itnode_->advertise(
		image_topic_name_,1,
		boost::bind( &labrob_gazebo_camera_utils::ImageConnect,this),
		boost::bind( &labrob_gazebo_camera_utils::ImageDisconnect,this),
		ros::VoidPtr(), &camera_queue_);
	
	ros::AdvertiseOptions camera_info_ao =
		ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
				camera_info_topic_name_,1,
				boost::bind( &labrob_gazebo_camera_utils::InfoConnect,this),
				boost::bind( &labrob_gazebo_camera_utils::InfoDisconnect,this),
				ros::VoidPtr(), &camera_queue_);
	camera_info_pub_ = rosnode_->advertise(camera_info_ao);
	
	ros::SubscribeOptions zoom_so =
		ros::SubscribeOptions::create<std_msgs::Float64>(
				"set_hfov",1,
				boost::bind( &labrob_gazebo_camera_utils::SetHFOV,this,_1),
				ros::VoidPtr(), &camera_queue_);
	cameraHFOVSubscriber_ = rosnode_->subscribe(zoom_so);
	
	ros::SubscribeOptions rate_so =
		ros::SubscribeOptions::create<std_msgs::Float64>(
				"set_update_rate",1,
				boost::bind( &labrob_gazebo_camera_utils::SetUpdateRate,this,_1),
				ros::VoidPtr(), &camera_queue_);
	cameraUpdateRateSubscriber_ = rosnode_->subscribe(rate_so);
	
	Init();
}

///////////////////////////////////////////////////////
///////////////// Increment count /////////////////////
///////////////////////////////////////////////////////
void labrob_gazebo_camera_utils::InfoConnect(){
	info_connect_count_++;
	// maintain for one more release for backwards compatibility with pr2_gazebo_plugins
	infoConnectCount = info_connect_count_;
}

///////////////////////////////////////////////////////
//////////////////// Decrement count //////////////////
///////////////////////////////////////////////////////
void labrob_gazebo_camera_utils::InfoDisconnect(){
	info_connect_count_--;
	// maintain for one more release for backwards compatibility with pr2_gazebo_plugins
	infoConnectCount = info_connect_count_;
}

///////////////////////////////////////////////////////
////////// Set Horizontal Field of View ///////////////
///////////////////////////////////////////////////////
void labrob_gazebo_camera_utils::SetHFOV(const std_msgs::Float64::ConstPtr& hfov){
	camera_->SetHFOV(hfov->data);
}

///////////////////////////////////////////////////////
//////////////// Set Update Rate //////////////////////
///////////////////////////////////////////////////////
void labrob_gazebo_camera_utils::SetUpdateRate(const std_msgs::Float64::ConstPtr& update_rate){
	parentSensor_->SetUpdateRate(update_rate->data);
}

///////////////////////////////////////////////////////
//////////////// Increment count //////////////////////
///////////////////////////////////////////////////////
void labrob_gazebo_camera_utils::ImageConnect(){
	image_connect_count_++;
	// maintain for one more release for backwards compatibility with pr2_gazebo_plugins
	imageConnectCount = image_connect_count_;
	parentSensor_->SetActive(true);
}

///////////////////////////////////////////////////////
////////////////// Decrement count ////////////////////
///////////////////////////////////////////////////////
void labrob_gazebo_camera_utils::ImageDisconnect(){
	image_connect_count_--;
	// maintain for one more release for backwards compatibility with pr2_gazebo_plugins
	imageConnectCount = image_connect_count_;
	if (image_connect_count_ <= 0)
		parentSensor_->SetActive(false);
}

///////////////////////////////////////////////////////
//////////////// Initialize the controller ////////////
///////////////////////////////////////////////////////
void labrob_gazebo_camera_utils::Init(){
	// set parent sensor update rate
	parentSensor_->SetUpdateRate(update_rate_);
	
	// prepare to throttle this plugin at the same rate
	// ideally, we should invoke a plugin update when the sensor updates,
	// have to think about how to do that properly later
	if (update_rate_ > 0.0)
		update_period_ = 1.0/update_rate_;
	else
		update_period_ = 0.0;
	
	
	// sensor generation off by default
	parentSensor_->SetActive(false);
	
	// set buffer size
	if (format_ == "L8"){
		type_ = sensor_msgs::image_encodings::MONO8;
		skip_ = 1;
	}else if (format_ == "R8G8B8"){
		type_ = sensor_msgs::image_encodings::RGB8;
		skip_ = 3;
	}else if (format_ == "B8G8R8"){
		type_ = sensor_msgs::image_encodings::BGR8;
		skip_ = 3;
	}else if (format_ == "BAYER_RGGB8"){
		ROS_INFO("bayer simulation maybe computationally expensive.");
		type_ = sensor_msgs::image_encodings::BAYER_RGGB8;
		skip_ = 1;
	}else if (format_ == "BAYER_BGGR8"){
		ROS_INFO("bayer simulation maybe computationally expensive.");
		type_ = sensor_msgs::image_encodings::BAYER_BGGR8;
		skip_ = 1;
	}else if (format_ == "BAYER_GBRG8"){
		ROS_INFO("bayer simulation maybe computationally expensive.");
		type_ = sensor_msgs::image_encodings::BAYER_GBRG8;
		skip_ = 1;
	}else if (format_ == "BAYER_GRBG8"){
		ROS_INFO("bayer simulation maybe computationally expensive.");
		type_ = sensor_msgs::image_encodings::BAYER_GRBG8;
		skip_ = 1;
	}else{
		ROS_ERROR("Unsupported Gazebo ImageFormat\n");
		type_ = sensor_msgs::image_encodings::BGR8;
		skip_ = 3;
	}
	
	/// Compute camera_ parameters if set to 0
	if (cx_prime_ == 0)
		cx_prime_ = ((double)width_+1.0) /2.0;
	if (cx_ == 0)
		cx_ = ((double)width_+1.0) /2.0;
	if (cy_ == 0)
		cy_ = ((double)height_+1.0) /2.0;
	
	
	double computed_focal_length = ((double)width_) / (2.0 *tan(camera_->GetHFOV().GetAsRadian()/2.0));
	if (focal_length_ == 0){
		focal_length_ = computed_focal_length;
	}else{
		if (!gazebo::math::equal(focal_length_, computed_focal_length)){ // check against float precision
			ROS_WARN("The <focal_length>[%f] you have provided for camera_ [%s] is inconsistent with specified image_width [%d] and HFOV [%f].   Please double check to see that focal_length = width_ / (2.0 * tan( HFOV/2.0 )), the explected focal_lengtth value is [%f], please update your camera_ model description accordingly.",
								focal_length_,parentSensor_->GetName().c_str(),width_,camera_->GetHFOV().GetAsRadian(),
								computed_focal_length);
		}
	}
	
	
	// start custom queue for camera_
	callback_queue_thread_ = boost::thread( boost::bind( &labrob_gazebo_camera_utils::CameraQueueThread,this ) );
}

///////////////////////////////////////////////////////
/////////// Put camera_ data to the interface /////////
///////////////////////////////////////////////////////
void labrob_gazebo_camera_utils::PutCameraData(const unsigned char *_src, common::Time &last_update_time){
	sensor_update_time_ = last_update_time;
	PutCameraData(_src);
}

void labrob_gazebo_camera_utils::PutCameraData(const unsigned char *_src){
	lock_.lock();
	// copy data into image
	image_msg_.header.frame_id = frame_name_;
	image_msg_.header.stamp.sec = sensor_update_time_.sec;
	image_msg_.header.stamp.nsec = sensor_update_time_.nsec;
	
	/// don't bother if there are no subscribers
	if (image_connect_count_ > 0){
		// copy from src to image_msg_
		fillImage(image_msg_,
				type_,
				height_,
				width_,
				skip_*width_,
				(void*)_src );
		
		// publish to ros
		image_pub_.publish(image_msg_);
	}
	
	lock_.unlock();
}


///////////////////////////////////////////////////////
/////////// Put camera_ data to the interface /////////
///////////////////////////////////////////////////////
void labrob_gazebo_camera_utils::PublishCameraInfo(common::Time &last_update_time){
	sensor_update_time_ = last_update_time;
	PublishCameraInfo();
}

void labrob_gazebo_camera_utils::PublishCameraInfo(){
	if (info_connect_count_ > 0){
		sensor_update_time_ = parentSensor_->GetLastUpdateTime();
		common::Time cur_time = world_->GetSimTime();
		if (cur_time - last_info_update_time_ >= update_period_){
			PublishCameraInfo(camera_info_pub_);
			last_info_update_time_ = cur_time;
		}
	}
}

void labrob_gazebo_camera_utils::PublishCameraInfo(ros::Publisher camera_info_publisher){
	sensor_msgs::CameraInfo camera_info_msg;
	// fill CameraInfo
	camera_info_msg.header.frame_id = frame_name_;
	camera_info_msg.header.stamp.sec = sensor_update_time_.sec;
	camera_info_msg.header.stamp.nsec = sensor_update_time_.nsec;
	camera_info_msg.height = height_;
	camera_info_msg.width  = width_;
	// distortion
#if ROS_VERSION_MINIMUM(1, 3, 0)
	camera_info_msg.distortion_model = "plumb_bob";
	camera_info_msg.D.resize(5);
#endif
	camera_info_msg.D[0] = distortion_k1_;
	camera_info_msg.D[1] = distortion_k2_;
	camera_info_msg.D[2] = distortion_k3_;
	camera_info_msg.D[3] = distortion_t1_;
	camera_info_msg.D[4] = distortion_t2_;
	// original camera_ matrix
	camera_info_msg.K[0] = focal_length_;
	camera_info_msg.K[1] = 0.0;
	camera_info_msg.K[2] = cx_;
	camera_info_msg.K[3] = 0.0;
	camera_info_msg.K[4] = focal_length_;
	camera_info_msg.K[5] = cy_;
	camera_info_msg.K[6] = 0.0;
	camera_info_msg.K[7] = 0.0;
	camera_info_msg.K[8] = 1.0;
	// rectification
	camera_info_msg.R[0] = 1.0;
	camera_info_msg.R[1] = 0.0;
	camera_info_msg.R[2] = 0.0;
	camera_info_msg.R[3] = 0.0;
	camera_info_msg.R[4] = 1.0;
	camera_info_msg.R[5] = 0.0;
	camera_info_msg.R[6] = 0.0;
	camera_info_msg.R[7] = 0.0;
	camera_info_msg.R[8] = 1.0;
	// camera_ projection matrix (same as camera_ matrix due to lack of distortion/rectification) (is this generated?)
	camera_info_msg.P[0] = focal_length_;
	camera_info_msg.P[1] = 0.0;
	camera_info_msg.P[2] = cx_;
	camera_info_msg.P[3] = -focal_length_ * hack_baseline_;
	camera_info_msg.P[4] = 0.0;
	camera_info_msg.P[5] = focal_length_;
	camera_info_msg.P[6] = cy_;
	camera_info_msg.P[7] = 0.0;
	camera_info_msg.P[8] = 0.0;
	camera_info_msg.P[9] = 0.0;
	camera_info_msg.P[10] = 1.0;
	camera_info_msg.P[11] = 0.0;
	
	camera_info_publisher.publish(camera_info_msg);
}


///////////////////////////////////////////////////////
///////// Put camera_ data to the interface ///////////
///////////////////////////////////////////////////////
void labrob_gazebo_camera_utils::CameraQueueThread(){
	static const double timeout = 0.001;
	
	while (rosnode_->ok()){
		/// publish CameraInfo
		PublishCameraInfo();
		
		/// take care of callback queue
		camera_queue_.callAvailable(ros::WallDuration(timeout));
	}
}

}