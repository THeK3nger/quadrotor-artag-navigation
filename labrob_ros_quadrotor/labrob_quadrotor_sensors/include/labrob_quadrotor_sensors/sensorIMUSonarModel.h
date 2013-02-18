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
 * Desc: model information for gazebo IMU
 * Author: Marco Cognetti
 * Date: 23 October 2012
 * SVN: $Id$
 */
//=================================================================================================
#ifndef QUADROTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H
#define QUADROTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H

#include <gazebo/sdf/sdf.h>

namespace gazebo {

	template <typename T>
	class SensorModel_ {
		public:
			SensorModel_();
			virtual ~SensorModel_();
			
			virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = std::string());
			
			virtual T operator()(const T& value) const { return value + current_error_; }
			virtual T operator()(const T& value, double dt) { return value + update(dt); }
			
			virtual T update(double dt);
			virtual void reset(const T& value = T());
			
			virtual const T& getCurrentError() const { return current_error_; }
			virtual const T& getCurrentDrift() const { return current_drift_; }
			
			virtual void setCurrentDrift(const T& new_drift) { current_drift_ = new_drift; }
			
		public:
			T offset;
			T drift;
			T drift_frequency;
			T gaussian_noise;
			
		private:
			T current_drift_;
			T current_error_;
	};
	
	template <typename T>
	SensorModel_<T>::SensorModel_() : offset(), drift(), drift_frequency(), gaussian_noise(){
		drift_frequency = 1.0/3600.0;
		reset();
	}
	
	template <typename T> SensorModel_<T>::~SensorModel_(){
	}
	
	template <typename T>
	void SensorModel_<T>::Load(sdf::ElementPtr _sdf, const std::string& prefix){
		sdf::ElementPtr _offset, _drift, _drift_frequency, _gaussian_noise;
		
		if (prefix.empty()) {
			_offset          = _sdf->GetElement("offset");
			_drift           = _sdf->GetElement("drift");
			_drift_frequency = _sdf->GetElement("driftFrequency");
			_gaussian_noise  = _sdf->GetElement("GaussianNoiseSigsa");
		} else {
			_offset          = _sdf->GetElement(prefix + "Offset");
			_drift           = _sdf->GetElement(prefix + "Drift");
			_drift_frequency = _sdf->GetElement(prefix + "DriftFrequency");
			_gaussian_noise  = _sdf->GetElement(prefix + "GaussianNoiseSigsa");
		}
		
		if (_offset          && !_offset->GetValue()->Get(offset))                   offset = _offset->GetValueDouble();
		if (_drift           && !_drift->GetValue()->Get(drift))                     drift = _drift->GetValueDouble();
		if (_drift_frequency && !_drift_frequency->GetValue()->Get(drift_frequency)) drift_frequency = _drift_frequency->GetValueDouble();
		if (_gaussian_noise  && !_gaussian_noise->GetValue()->Get(gaussian_noise))   gaussian_noise = _gaussian_noise->GetValueDouble();
	}
	
	namespace {
		template <typename T>
		static inline T SensorModelGaussianKernel(T mu, T sigma){
			// using Box-Muller transform to generate two independent standard normally disbributed normal variables
			// see wikipedia
			T U = (T)rand()/(T)RAND_MAX; // normalized uniform random variable
			T V = (T)rand()/(T)RAND_MAX; // normalized uniform random variable
			T X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
			X = sigma * X + mu;
			return X;
		}
		
		template <typename T>
		static inline T SensorModelInternalUpdate(T& current_drift, T drift, T drift_frequency, T offset, T gaussian_noise, double dt){
			current_drift = current_drift - dt * (current_drift * drift_frequency + SensorModelGaussianKernel(T(), sqrt(2*drift_frequency)*drift));
			return offset + current_drift + SensorModelGaussianKernel(T(), gaussian_noise);
		}
	}
	
	template <typename T>
	T SensorModel_<T>::update(double dt){
		for(std::size_t i = 0; i < current_error_.size(); ++i) current_error_[i] = current_error_ = SensorModelInternalUpdate(current_drift_[i], drift[i], drift_frequency[i], offset[i], gaussian_noise[i], dt);
		return current_error_;
	}
	
	template <>
	double SensorModel_<double>::update(double dt){
		current_error_ = SensorModelInternalUpdate(current_drift_, drift, drift_frequency, offset, gaussian_noise, dt);
		return current_error_;
	}
	
	template <>
	math::Vector3 SensorModel_<math::Vector3>::update(double dt){
		current_error_.x = SensorModelInternalUpdate(current_drift_.x, drift.x, drift_frequency.x, offset.x, gaussian_noise.x, dt);
		current_error_.y = SensorModelInternalUpdate(current_drift_.y, drift.y, drift_frequency.y, offset.y, gaussian_noise.y, dt);
		current_error_.z = SensorModelInternalUpdate(current_drift_.z, drift.z, drift_frequency.z, offset.z, gaussian_noise.z, dt);
		return current_error_;
	}
	
	template <typename T>
	void SensorModel_<T>::reset(const T& value){
		current_drift_ = T();
		current_error_ = value;
	}
	
	typedef SensorModel_<double> SensorModel;
	typedef SensorModel_<math::Vector3> SensorModel3;
	
}

#endif // QUADROTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H
