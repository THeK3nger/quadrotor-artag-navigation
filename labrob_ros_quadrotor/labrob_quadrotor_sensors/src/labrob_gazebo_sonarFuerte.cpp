//=================================================================================================
// Copyright (c) 2012, Marco Cognetti
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.

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

#include <labrob_quadrotor_sensors/labrob_gazebo_sonarFuerte.h>
#include "common/Events.hh"
#include "physics/physics.h"
#include "sensors/RaySensor.hh"

#include <limits>

namespace gazebo {
	
	////////////////////////////////////////////
	///////////// Constructor //////////////////
	////////////////////////////////////////////
	labrob_gazebo_sonar::labrob_gazebo_sonar(){
	}
	
	////////////////////////////////////////////
	///////////// Destructor ///////////////////
	////////////////////////////////////////////
	labrob_gazebo_sonar::~labrob_gazebo_sonar(){
		sensor_->SetActive(false);
		event::Events::DisconnectWorldUpdateStart(updateConnection);
		node_handle_->shutdown();
		delete node_handle_;
	}
	
	////////////////////////////////////////////
	/////////// Load the controller ////////////
	////////////////////////////////////////////
	void labrob_gazebo_sonar::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
		// Get then name of the parent sensor
		sensor_ = boost::shared_dynamic_cast<sensors::RaySensor>(_sensor);
		if (!sensor_){
			gzthrow("labrob_gazebo_sonar requires a Ray Sensor as its parent");
			return;
		}
		
		// Get the world name.
		std::string worldName = sensor_->GetWorldName();
		world = physics::get_world(worldName);
		
		// Load parameters
		// Namespace
		if (!_sdf->HasElement("robotNamespace"))
			namespace_.clear();
		else
			namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
		
		// Set namespace
		if (_sdf->HasElement("setNamespace")){
			std::string setNm = _sdf->GetElement("setNamespace")->GetValueString();
			std::string pn = sensor_->GetParentName();
			if(setNm == "1" || setNm == "true"){
				std::size_t pos;
				pos = pn.find(":");
	// 			std::cout << "pos: " << pos << " pnSize: " << std::string::npos << std::endl;
				if(pos!=std::string::npos){
					pn = pn.substr(0,pos);
					namespace_ = pn + "/";
					std::cout << "robotNamespace sonar: " << namespace_ << std::endl;
				}else{
					ROS_WARN("A namespace can not include :! Check if the actual namespace cointans it!");
					namespace_ = pn;
				}
			}
		}
		
		// Gaussian noise
		if (!_sdf->HasElement("gaussianNoiseSigma"))
			gaussianNoise = 0.0;
		else
			gaussianNoise = _sdf->GetElement("gaussianNoiseSigma")->GetValueDouble();
		
		// Frame id 
		if (!_sdf->HasElement("frameId"))
			frame_id_ = "";
		else
			frame_id_ = _sdf->GetElement("frameId")->GetValueString();
		
		// Topic name
		if (!_sdf->HasElement("topicName"))
			topic_ = "sonar";
		else
			topic_ = _sdf->GetElement("topicName")->GetValueString();
		
		if (!_sdf->HasElement("topicNameLaser"))
			topicLs_ = "sonarLaser";
		else
			topicLs_ = _sdf->GetElement("topicNameLaser")->GetValueString();
		
		sensor_model_.Load(_sdf);
		sensor_model_.gaussian_noise = gaussianNoise;
		
		double update_rate = 0.0;
		if (_sdf->HasElement("updateRate")) update_rate = _sdf->GetElement("updateRate")->GetValueDouble();
		update_period = update_rate > 0.0 ? 1.0/update_rate : 10.0;
		
		std::cout << "upRate SONAR: " << update_rate << " per: " << update_period << std::endl;
		
		range_.header.frame_id = frame_id_;
		range_.radiation_type = sensor_msgs::Range::ULTRASOUND;
		range_.field_of_view = std::min(fabs((sensor_->GetAngleMax() - sensor_->GetAngleMin()).GetAsRadian()), fabs((sensor_->GetVerticalAngleMax() - sensor_->GetVerticalAngleMin()).GetAsRadian()));
		range_.max_range = sensor_->GetRangeMax();
		range_.min_range = sensor_->GetRangeMin();
		
		// Fill laser msg
		ls.header.frame_id = frame_id_;
		ls.angle_max = sensor_->GetAngleMax().GetAsRadian();
		ls.angle_min = sensor_->GetAngleMin().GetAsRadian();
		ls.range_min = range_.min_range;
		ls.range_max = range_.max_range;
		
		// start ros node
		if (!ros::isInitialized()){
			int argc = 0;
			char** argv = NULL;
			ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
		}
		
		node_handle_ = new ros::NodeHandle(namespace_);
		publisher_   = node_handle_->advertise<sensor_msgs::Range>(topic_, 1);
		publisherLs_ = node_handle_->advertise<sensor_msgs::LaserScan>(topicLs_, 1);
		
		Reset();
		updateConnection = sensor_->GetLaserShape()->ConnectNewLaserScans(boost::bind(&labrob_gazebo_sonar::Update, this));
		
		// activate RaySensor
		sensor_->SetActive(true);
		
	}
	
	void labrob_gazebo_sonar::Reset(){
		sensor_model_.reset();
	}
	
	///////////////////////////////////
	////// Update the controller //////
	///////////////////////////////////
	void labrob_gazebo_sonar::Update(){
		
		common::Time sim_time = world->GetSimTime();
		std::cout << "1: " << last_time.Double() << " 2: " << update_period << " 3: " << sim_time.Double() << std::endl;
		if (sim_time - last_time >= update_period){
			double dt = (sim_time - last_time).Double();
			
// 			if (last_time + update_period > sim_time) return;
			
			// activate RaySensor if it is not yet active
			if (!sensor_->IsActive()) sensor_->SetActive(true);
			
			range_.header.stamp.sec  = (world->GetSimTime()).sec;
			range_.header.stamp.nsec = (world->GetSimTime()).nsec;
			
			// find ray with minimal range
			range_.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();
			
			// Fill laser scan
			ls.ranges.clear();
			ls.header.stamp.sec  = range_.header.stamp.sec;
			ls.header.stamp.nsec = range_.header.stamp.nsec;
			ls.angle_increment   = (ls.angle_max - ls.angle_min)/((double)(sensor_->GetRangeCount() -1));
			ls.time_increment    = 0; // instantaneous simulator scan
			ls.scan_time         = 0; // FIXME: what's this?
			
			for(int i = 0; i < sensor_->GetRangeCount(); ++i) {
				double ray = sensor_->GetLaserShape()->GetRange(i);
				if (ray < range_.range){ 
					range_.range = ray;
					ls.ranges.push_back(ray+ GaussianKernel(0,gaussianNoise));
				}
			}
			
			
			// add Gaussian noise (and limit to min/max range)
			if (range_.range < range_.max_range) {
				range_.range += sensor_model_.update(dt);
				if (range_.range < range_.min_range) range_.range = range_.min_range;
				if (range_.range > range_.max_range) range_.range = range_.max_range;
			}
			
			publisher_.publish(range_);
			
			// Laser msg
			publisherLs_.publish(ls);
			
			// save last time stamp
			last_time = sim_time;
		}
	}
	
	/////////////////////////////////////////////
	///////// Utility for adding noise //////////
	/////////////////////////////////////////////
	double labrob_gazebo_sonar::GaussianKernel(double mu,double sigma){
		// using Box-Muller transform to generate two independent standard normally disbributed normal variables
		// Check wikipedia
		double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
		double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
		double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
		//double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
		// we'll just use X
		// scale to our mu and sigma
		X = sigma * X + mu;
		return X;
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_SENSOR_PLUGIN(labrob_gazebo_sonar)
	
} // namespace gazebo
