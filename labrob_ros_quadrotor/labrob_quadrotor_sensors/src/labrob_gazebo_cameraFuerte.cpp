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
	Desc: labrob_gazebo_camera plugin for simulating cameras in Gazebo
	Author: Marco Cognetti
	Date: 22 October 2012
	SVN info: $Id$
@htmlinclude manifest.html
@b labrob_gazebo_camera plugin broadcasts ROS Image messages
*/

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <labrob_quadrotor_sensors/labrob_gazebo_cameraFuerte.h>

#include "sensors/Sensor.hh"
#include "sdf/interface/SDF.hh"
#include "sensors/SensorTypes.hh"

// for creating PointCloud2 from pcl point cloud
#include "pcl/ros/conversions.h"

#include "tf/tf.h"

namespace gazebo{

////////////////////////////////////////////
////////////// Constructor /////////////////
////////////////////////////////////////////
labrob_gazebo_camera::labrob_gazebo_camera(){
	point_cloud_connect_count_ = 0;
}

////////////////////////////////////////////
///////////////// Destructor ///////////////
////////////////////////////////////////////
labrob_gazebo_camera::~labrob_gazebo_camera(){
}

////////////////////////////////////////////
//////////// Increment count ///////////////
////////////////////////////////////////////
void labrob_gazebo_camera::InfoConnect(){
	info_connect_count_++;
}

////////////////////////////////////////////
//////////// Decrement count ///////////////
////////////////////////////////////////////
void labrob_gazebo_camera::InfoDisconnect(){
	info_connect_count_--;
}

////////////////////////////////////////////
//////////// Load the controller ///////////
////////////////////////////////////////////
void labrob_gazebo_camera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf){ 
	DepthCameraPlugin::Load(_parent, _sdf);
	// copying from DepthCameraPlugin into GazeboRosCameraUtils
	parentSensor_ = parentSensor;
	width_        = width;
	height_       = height;
	depth_        = depth;
	format_       = format;
	camera_       = depthCamera;
	
	labrob_gazebo_camera_utils::Load(_parent, _sdf);
	
	// using a different default
	if (!_sdf->GetElement("imageTopicName"))
		image_topic_name_ = "ir/image_raw";
	if (!_sdf->HasElement("cameraInfoTopicName"))
		camera_info_topic_name_ = "ir/camera_info";
	
	// point cloud stuff
	if (!_sdf->GetElement("pointCloudTopicName"))
		point_cloud_topic_name_ = "points";
	else
		point_cloud_topic_name_ = _sdf->GetElement("pointCloudTopicName")->GetValueString();
	
	// depth image stuff
	if (!_sdf->GetElement("depthImageTopicName"))
		depth_image_topic_name_ = "depth/image_raw";
	else
		depth_image_topic_name_ = _sdf->GetElement("depthImageTopicName")->GetValueString();
	
	if (!_sdf->GetElement("depthImageCameraInfoTopicName"))
		depth_image_camera_info_topic_name_ = "depth/camera_info";
	else
		depth_image_camera_info_topic_name_ = _sdf->GetElement("depthImageCameraInfoTopicName")->GetValueString();
	
	if (!_sdf->GetElement("pointCloudCutoff"))
		point_cloud_cutoff_ = 0.4;
	else
		point_cloud_cutoff_ = _sdf->GetElement("pointCloudCutoff")->GetValueDouble();
	
	ros::AdvertiseOptions point_cloud_ao =
		ros::AdvertiseOptions::create<sensor_msgs::PointCloud2 >(
			point_cloud_topic_name_,1,
			boost::bind( &labrob_gazebo_camera::PointCloudConnect,this),
			boost::bind( &labrob_gazebo_camera::PointCloudDisconnect,this),
			ros::VoidPtr(), &camera_queue_);
	point_cloud_pub_ = rosnode_->advertise(point_cloud_ao);
	
	ros::AdvertiseOptions depth_image_ao =
		ros::AdvertiseOptions::create< sensor_msgs::Image >(
			depth_image_topic_name_,1,
			boost::bind( &labrob_gazebo_camera::PointCloudConnect,this),
			boost::bind( &labrob_gazebo_camera::PointCloudDisconnect,this),
			ros::VoidPtr(), &camera_queue_);
	depth_image_pub_ = rosnode_->advertise(depth_image_ao);
	
	ros::AdvertiseOptions depth_image_camera_info_ao =
		ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
				depth_image_camera_info_topic_name_,1,
				boost::bind( &labrob_gazebo_camera::InfoConnect,this),
				boost::bind( &labrob_gazebo_camera::InfoDisconnect,this),
				ros::VoidPtr(), &camera_queue_);
	depth_image_camera_info_pub_ = rosnode_->advertise(depth_image_camera_info_ao);
	
}

////////////////////////////////////////////
////////////// Increment count /////////////
////////////////////////////////////////////
void labrob_gazebo_camera::PointCloudConnect(){
	point_cloud_connect_count_++;
	image_connect_count_++;
	parentSensor->SetActive(true);
}

////////////////////////////////////////////
////////////// Decrement count /////////////
////////////////////////////////////////////
void labrob_gazebo_camera::PointCloudDisconnect(){
	point_cloud_connect_count_--;
	image_connect_count_--;
	if (point_cloud_connect_count_ <= 0)
		parentSensor->SetActive(false);
}

////////////////////////////////////////////
///////////// Update the controller ////////
////////////////////////////////////////////
void labrob_gazebo_camera::OnNewDepthFrame(const float *_image,
		unsigned int _width, unsigned int _height, unsigned int _depth,
		const std::string &_format){
	depth_sensor_update_time_ = parentSensor->GetLastUpdateTime();
	if (!parentSensor->IsActive()){
		if (point_cloud_connect_count_ > 0)
			// do this first so there's chance for sensor to run 1 frame after activate
			parentSensor->SetActive(true);
	}else{
		if (point_cloud_connect_count_ > 0)
			FillPointdCloud(_image);
	}
}

////////////////////////////////////////////
///////// Update the controller ////////////
////////////////////////////////////////////
void labrob_gazebo_camera::OnNewRGBPointCloud(const float *_pcd,
		unsigned int _width, unsigned int _height, unsigned int _depth,
		const std::string &_format){
	depth_sensor_update_time_ = parentSensor->GetLastUpdateTime();
	if (!parentSensor->IsActive()){
		if (point_cloud_connect_count_ > 0)
			// do this first so there's chance for sensor to run 1 frame after activate
			parentSensor->SetActive(true);
	}else{
		if (point_cloud_connect_count_ > 0){
			lock_.lock();
			point_cloud_msg_.header.frame_id = frame_name_;
			point_cloud_msg_.header.stamp.sec = depth_sensor_update_time_.sec;
			point_cloud_msg_.header.stamp.nsec = depth_sensor_update_time_.nsec;
			point_cloud_msg_.width = width;
			point_cloud_msg_.height = height;
			point_cloud_msg_.row_step = point_cloud_msg_.point_step * width;
			
			pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
			point_cloud.points.resize(0);
			point_cloud.is_dense = true;
			
			for (unsigned int i = 0; i < _width; i++){
				for (unsigned int j = 0; j < _height; j++){
					unsigned int index = (j * _width) + i;
					pcl::PointXYZRGB point;
					point.x = _pcd[4 * index];
					point.y = _pcd[4 * index + 1];
					point.z = _pcd[4 * index + 2];
					point.rgb = _pcd[4 * index + 3];
					point_cloud.points.push_back(point);
					if (i == _width /2 && j == _height / 2){
						uint32_t rgb = *reinterpret_cast<int*>(&point.rgb);
						uint8_t r = (rgb >> 16) & 0x0000ff;
						uint8_t g = (rgb >> 8)  & 0x0000ff;
						uint8_t b = (rgb)       & 0x0000ff;
						std::cerr << (int)r << " " << (int)g << " " << (int)b << "\n";
					}
				}
			}
			point_cloud.header = point_cloud_msg_.header;
			pcl::toROSMsg(point_cloud, point_cloud_msg_);
			
			point_cloud_pub_.publish(point_cloud_msg_);
			lock_.unlock();
		}
	}
}

////////////////////////////////////////////
///////// Update the controller ////////////
////////////////////////////////////////////
void labrob_gazebo_camera::OnNewImageFrame(const unsigned char *_image,
		unsigned int _width, unsigned int _height, unsigned int _depth,
		const std::string &_format){
	//ROS_ERROR("camera_ new frame %s %s",parentSensor_->GetName().c_str(),frame_name_.c_str());
	sensor_update_time_ = parentSensor->GetLastUpdateTime();
	
	if (!parentSensor->IsActive()){
		if (image_connect_count_ > 0)
			// do this first so there's chance for sensor to run 1 frame after activate
			parentSensor->SetActive(true);
	}else{
		if (image_connect_count_ > 0)
			PutCameraData(_image);
	}
}

////////////////////////////////////////////
////// Put camera data to the interface ////
////////////////////////////////////////////
void labrob_gazebo_camera::FillPointdCloud(const float *_src){
	lock_.lock();
	
	point_cloud_msg_.header.frame_id = frame_name_;
	point_cloud_msg_.header.stamp.sec = depth_sensor_update_time_.sec;
	point_cloud_msg_.header.stamp.nsec = depth_sensor_update_time_.nsec;
	point_cloud_msg_.width = width;
	point_cloud_msg_.height = height;
	point_cloud_msg_.row_step = point_cloud_msg_.point_step * width;
	
	///copy from depth to pointCloudMsg
	FillPointCloudHelper(point_cloud_msg_,
								height,
								width,
								1,
								(void*)_src );
	
	point_cloud_pub_.publish(point_cloud_msg_);
	
	// copy data into image
	depth_image_msg_.header.frame_id = frame_name_;
	depth_image_msg_.header.stamp.sec = depth_sensor_update_time_.sec;
	depth_image_msg_.header.stamp.nsec = depth_sensor_update_time_.nsec;
	
	///copy from depth to depth image message
	FillDepthImageHelper(depth_image_msg_,
								height,
								width,
								1,
								(void*)_src );
	
	depth_image_pub_.publish(depth_image_msg_);
	
	lock_.unlock();
}

////////////////////////////////////////////
////////// Fill depth information //////////
////////////////////////////////////////////
bool labrob_gazebo_camera::FillPointCloudHelper(
		sensor_msgs::PointCloud2 &point_cloud_msg,
		uint32_t rows_arg, uint32_t cols_arg,
		uint32_t step_arg, void* data_arg){
	pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
	
	point_cloud.points.resize(0);
	point_cloud.is_dense = true;
	
	float* toCopyFrom = (float*)data_arg;
	int index = 0;
	
	double hfov = parentSensor->GetDepthCamera()->GetHFOV().GetAsRadian();
	double fl = ((double)width) / (2.0 *tan(hfov/2.0));
	
	// convert depth to point cloud
	for (uint32_t j=0; j<rows_arg; j++){
		double pAngle;
		if (rows_arg>1) pAngle = atan2( (double)j - 0.5*(double)(rows_arg-1), fl);
		else            pAngle = 0.0;
		
		for (uint32_t i=0; i<cols_arg; i++){
			double yAngle;
			if (cols_arg>1) yAngle = atan2( (double)i - 0.5*(double)(cols_arg-1), fl);
			else            yAngle = 0.0;
			
			double depth = toCopyFrom[index++];
			
			// in optical frame
			// hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
			// to urdf, where the *_optical_frame should have above relative
			// rotation from the physical camera *_frame
			pcl::PointXYZRGB point;
			point.x      = depth * tan(yAngle);
			point.y      = depth * tan(pAngle);
			if(depth > point_cloud_cutoff_){
				point.z    = depth;
			}else{ //point in the unseeable range
				point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
				point_cloud.is_dense = false;
			}
			
			// put image color data for each point
			uint8_t*  image_src = (uint8_t*)(&(image_msg_.data[0]));
			if (image_msg_.data.size() == rows_arg*cols_arg*3){
				// color
				point.r = image_src[i*3+j*cols_arg*3+0];
				point.g = image_src[i*3+j*cols_arg*3+1];
				point.b = image_src[i*3+j*cols_arg*3+2];
			}else if (image_msg_.data.size() == rows_arg*cols_arg){
				// mono (or bayer?  @todo; fix for bayer)
				point.r = image_src[i+j*cols_arg];
				point.g = image_src[i+j*cols_arg];
				point.b = image_src[i+j*cols_arg];
			}else{
				// no image
				point.r = 0;
				point.g = 0;
				point.b = 0;
			}
			
			point_cloud.points.push_back(point);
		}
	}
	
	point_cloud.header = point_cloud_msg.header;
	pcl::toROSMsg(point_cloud, point_cloud_msg);
	return true;
}

////////////////////////////////////////////
//////// Fill depth information ////////////
////////////////////////////////////////////
bool labrob_gazebo_camera::FillDepthImageHelper(
		sensor_msgs::Image& image_msg,
		uint32_t rows_arg, uint32_t cols_arg,
		uint32_t step_arg, void* data_arg){
	image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	image_msg.height = rows_arg;
	image_msg.width = cols_arg;
	image_msg.step = 1;
	image_msg.data.resize(rows_arg * cols_arg * sizeof(float));
	image_msg.is_bigendian = 0;
	
	const float bad_point = std::numeric_limits<float>::quiet_NaN();
	
	float* dest = (float*)(&(image_msg.data[0]));
	float* toCopyFrom = (float*)data_arg;
	int index = 0;
	
	// convert depth to point cloud
	for (uint32_t j = 0; j < rows_arg; j++){
		for (uint32_t i = 0; i < cols_arg; i++){
			float depth = 0;
			for (uint32_t s = 0; s < step_arg; s++) // if depth > 1 (e.g. rgb), average
				depth += toCopyFrom[index++];
			depth = depth / (float)step_arg;
			
			if (depth > point_cloud_cutoff_){
				dest[i + j * cols_arg] = depth;
			}else{ //point in the unseeable range
				dest[i + j * cols_arg] = bad_point;
			}
		}
	}
	return true;
}

void labrob_gazebo_camera::PublishCameraInfo(){
	ROS_DEBUG("publishing depth camera info, then camera info");
	labrob_gazebo_camera_utils::PublishCameraInfo();
	PublishCameraInfo(depth_image_camera_info_pub_);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(labrob_gazebo_camera);

}