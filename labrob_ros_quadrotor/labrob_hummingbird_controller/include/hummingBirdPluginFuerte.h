#ifndef LABROB_HUBI_PLUGIN_HH
#define LABROB_HUBI_PLUGIN_HH

#include "gazebo.hh"
#include <sdf/interface/Param.hh>
#include <sdf/interface/SDF.hh>
#include <physics/physics.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/String.h>
#include <common/common.h>
#include <ros/ros.h>
#include <ar_pose/ARMarkers.h>

#include <fstream>
#include <iostream>
#include <armadillo>

namespace gazebo{
class Joint;
class Entity;

class HummingBirdPlugin : public ModelPlugin{
private:
    // ROS nodes variables
    ros::NodeHandle rosNodeHandle;
    ros::Subscriber inputSubscriber;
    ros::Subscriber artagSubscriber;
    ros::Subscriber imuSubscriber;
    ros::Subscriber switchSubscriber;

	double arX,arY,arZ;
	double imuX, imuY, imuZ; //velocità letta dalla imu
	int timer, timerSwitch;
	int arIndex;
	bool arFlag, switchFlag, smoothFlag;
	std::ofstream fileName;

	private:
		double total_mass;
		physics::ModelPtr parent_model;
		physics::WorldPtr world;
		physics::LinkPtr  body_main;
		physics::LinkPtr *body_motors;
		
		physics::JointPtr *joints;
		
		// Used in computation
		// Current rotation
		math::Vector3 current_rotation_euler;
		math::Vector3 current_rotation_vel;
		
		// Auxiliar variables
		math::Vector3 desired_position;
		math::Vector3 initial_position;
		math::Vector3 current_position;
		math::Vector3 GPS_position;
		math::Vector3 current_velocity;
		
		// Control variables
		double p_gain[3];
		double d_gain[3];
		
		// see attitude_control()
		double roll, pitch, yaw;
		double roll_ref, pitch_ref, yaw_ref, thrust_ref;
		double roll_torque, pitch_torque, yaw_torque;  
		math::Vector3 p_error;
		math::Vector3 v_error;
		
		double Thrust;
		
		// Update rate of the sensor
		double update_rate;
		// Update period of the sensor
		double update_period;
		// Elapsed time
		common::Time last_update_time;
		
		
		// Actual iteration
		event::ConnectionPtr updateConnection;
		
		// used in set_motor_speed()
		double prop_speed[4];
		double prop_speed_prev[4];
		math::Vector3 motor_force[4];
		
		std::string robotNamespace;
		int selected_input;
		
		// Setting ROS name space, parameters and functions
		bool matlabMode, enablePropellers     , enableAttitudeControl,
		                 enablePositionControl, enableAbsoluteError;
		double motorMaxSpeed, thrustScale, propDragCoeff, propThrustCoeff;
		std::string inputType;
		bool recordLog;
		
		// Conversion matrix and vectors for propeller simulation
		arma::Mat<double>::fixed<4,4> torques2omega;   	//! Matrix to convert from torques to propellers rotational speeds
		arma::Col<double>::fixed<4> torques;           	//! Vector of torques (to be converted with torques2omega)
		arma::Col<double>::fixed<4> omega_square;      	//! Vector of propellers rotational speeds
		arma::Mat<double>::fixed<3,3> abs2rel;         	//! Roll pitch yaw matric transformation
		arma::Mat<double>::fixed<3,4> mot2trasf;       	//! Matrix to be transformed via roll pitch yaw (i.e. motor forces)
		arma::Col<double>::fixed<3>   tmpVec;          	//! Temporary structure to perform computation with Armadillo
		
		std::ofstream logFile;
	private:
		/// Low Level controller Loop and functions
		void LowLevelLoop();
		void IMU_update_angles();
		void IMU_update_accelerations(){};
		void update_barometer(){};
		void attitude_control();
		void set_motor_speed();
		
		/// High Level controller Loop
		void HighLevelLoop();
		void update_sonar(){}
		void height_Control_gravity_comp();
		void height_Control_2(){}
		void receive_commands();
		
		/// Position Control
		void position_control();
		
		void update_ground_truth();
		
		void PublishData(){}
		void record_log_file();
		void   open_log_file();
		void  close_log_file();
		double zero_check(double v);
		void normalize_angle(double& a);
		void normalize_angle(math::Vector3& v);
		
		// Input callbacks
		void positionInputCallback(const geometry_msgs::Twist::ConstPtr & msg);
		void torquesInputCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
		void actualInputCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
		void serialInputCallback(const std_msgs::String::ConstPtr& msg);
		void switchCallback(const std_msgs::String::ConstPtr& msg);
		void artagCallback(const ar_pose::ARMarkers::ConstPtr& msg);
		void imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

		math::Vector3 linearInterpol(double s);
		bool markerSearch(int id, const ar_pose::ARMarkers::ConstPtr& msg, math::Vector3& markerPose);

	public:
		HummingBirdPlugin();
		~HummingBirdPlugin();
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
		void Init();
		protected: void UpdateChild();
		protected: virtual void FiniChild();
};

} // close namespace

#endif
