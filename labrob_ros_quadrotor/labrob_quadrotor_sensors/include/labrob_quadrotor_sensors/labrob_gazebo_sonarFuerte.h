//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_SONAR_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_SONAR_H

#include "common/Plugin.hh"

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <sensor_msgs/Range.h>
#include "sensorIMUSonarModel.h"
#include <sensor_msgs/LaserScan.h>

namespace gazebo{
	
	class labrob_gazebo_sonar : public SensorPlugin{
		public:
			labrob_gazebo_sonar();
			virtual ~labrob_gazebo_sonar();
		
		protected:
			virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
			virtual void Reset();
			virtual void Update();
			
			/// \brief Adding noise using Gaussian kernel
			/// \param[in].mu Mean of the noise
			/// \param[in].sigma Covariance of the noise
			double GaussianKernel(double mu,double sigma);
		
		private:
			/// \brief The parent World
			physics::WorldPtr world;
			
			/// \brief Period in seconds of the update time
			double update_period;
			
			/// \brief Gaussian noise intensity
			float gaussianNoise;
			
			sensors::RaySensorPtr sensor_;
			
			ros::NodeHandle* node_handle_;
			ros::Publisher publisher_;
			ros::Publisher publisherLs_;
			
			sensor_msgs::Range range_;
			sensor_msgs::LaserScan ls;
			
			std::string namespace_;
			std::string topic_;
			std::string topicLs_;
			std::string frame_id_;
			
			SensorModel sensor_model_;
			
			/// \brief save last_time
			common::Time last_time;
			
			// Pointer to the update event connection
			event::ConnectionPtr updateConnection;
	};
	
} // namespace gazebo

#endif // QUADROTOR_GAZEBO_PLUGINS_GAZEBO_ROS_SONAR_H