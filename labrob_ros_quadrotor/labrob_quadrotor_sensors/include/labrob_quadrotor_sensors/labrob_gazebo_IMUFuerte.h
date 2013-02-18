// This code is based on the original gazebo_ros_imu plugin by Sachin Chitta and John Hsu:
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
* Desc: 3D position interface.
* Author: Marco Cognetti
* Date: 23 October 2012
* SVN: $Id$
*/
//=================================================================================================


#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H

// #define USE_CBQ
#ifdef USE_CBQ
	#include <ros/callback_queue.h>
	#include <ros/advertise_options.h>
#endif

#include "common/Plugin.hh"

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_srvs/Empty.h>
#include "setBias.h"
#include "sensorIMUSonarModel.h"

namespace gazebo{
	class labrob_gazebo_IMU : public ModelPlugin{
		public:
				/// \brief Constructor
				labrob_gazebo_IMU();
				
				/// \brief Destructor
				virtual ~labrob_gazebo_IMU();
		
		protected:
				virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
				virtual void Reset();
				virtual void Update();
		
		private:
				/// \brief The parent World
				physics::WorldPtr world;
				
				/// \brief The link referred to by this plugin
				physics::LinkPtr link;
				
				/// \brief pointer to ros node
				ros::NodeHandle* node_handle_;
				ros::Publisher pub_;
				
				/// \brief Publisher for the velocity
				ros::Publisher pubVel_;
				
				/// \brief ros message
				sensor_msgs::Imu imuMsg;
				
				/// \brief store link name
				std::string linkName;
				
				/// \brief frame id
				std::string frameId;
				
				/// \brief topic name for imu data
				std::string topicName;
				
				/// \brief topic name for velocity
				std::string topicNameVel;
				
				/// \brief Sensor models
				SensorModel3 accelModel;
				SensorModel3 rateModel;
				SensorModel headingModel;
				
				/// \brief A mutex to lock access to fields that are used in message callbacks
				boost::mutex lock;
				
				/// \brief save last_time
				common::Time last_time;
				common::Time update_period;
				
				/// \brief save current body/physics state
				math::Quaternion orientation;
				math::Vector3 velocity;
				math::Vector3 accel;
				math::Vector3 rate;
				math::Vector3 gravity;
				math::Vector3 gravity_body;
				
				/// \brief Gaussian noise generator
				double GaussianKernel(double mu,double sigma);
				
				/// \brief for setting ROS name space
				std::string robotNamespace;
				
				/// \brief call back when using service
				bool ServiceCallback(std_srvs::Empty::Request &req,
																			std_srvs::Empty::Response &res);
				ros::ServiceServer srv_;
				std::string serviceName;
				
				/// \brief Bias service callbacks
				bool SetAccelBiasCallback(quadrotor_gazebo_plugins::SetBias::Request &req, quadrotor_gazebo_plugins::SetBias::Response &res);
				bool SetRateBiasCallback(quadrotor_gazebo_plugins::SetBias::Request &req, quadrotor_gazebo_plugins::SetBias::Response &res);
				ros::ServiceServer accelBiasService;
				ros::ServiceServer rateBiasService;
	
	#ifdef USE_CBQ
			ros::CallbackQueue callback_queue_;
			void CallbackQueueThread();
			boost::thread callback_queue_thread_;
	#endif
				
			// Pointer to the update event connection
			event::ConnectionPtr updateConnection;
		};
}

#endif // QUADROTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H
