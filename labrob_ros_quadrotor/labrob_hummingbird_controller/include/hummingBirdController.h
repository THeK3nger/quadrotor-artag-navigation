#ifndef LABROB_HUBI_CONTROLLER_HH
#define LABROB_HUBI_CONTROLLER_HH

#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Model.hh>

// ROS 
#include <ros/ros.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include <iostream>
#include <armadillo>

namespace gazebo
{
class Joint;
class Entity;

class HummingBirdController : public Controller
{
  private:
		// ROS nodes variables
		ros::NodeHandle rosNodeHandle;
		ros::Subscriber inputSubscriber;

	private:
		double total_mass;
		Model	*parent_model;
		Body	*body_main;
		Body	**body_motors;
		
		Joint **joints;
		
		// Used in computation
		// Current rotation
		Vector3 current_rotation_euler;
		Vector3 current_rotation_vel;
		
		// Auxiliar variables
		Vector3 desired_position;
		Vector3 current_position;
		Vector3 GPS_position;
		Vector3 current_velocity;
		
		// Control variables
		double p_gain[3];
		double d_gain[3];
		
		// see attitude_control()
		double roll, pitch, yaw;
		double roll_ref, pitch_ref, yaw_ref, thrust_ref;
		double roll_torque, pitch_torque, yaw_torque;  
		Vector3 p_error;
		Vector3 v_error;
		
		double Thrust;
		
		// used in set_motor_speed()
		double prop_speed[4];
		double prop_speed_prev[4];
		Vector3 motor_force[4];
		
		std::string robotNamespace;
		int selected_input;
		
		// Setting ROS name space, parameters and functions
		ParamT<std::string> *inputType;
		ParamT<std::string> *robotNamespaceP;
		
		// Robot parameters
		ParamT<double> *armLength;   	//! Length of the arm (from the body to the motor)
		
		// Blades parameters
		ParamT<double> *propThrustCoeff; 	//! Thrust aerodynamic coefficient (propellers)
		ParamT<double> *propDragCoeff;   	//! Drag aerodynamic coefficient (propellers)
		ParamT<double> *motorMaxSpeed;   	//! Maximum speed for motors/propellers
		ParamT<double> *thrustScale;     	//! Thrust scaling factor (It should be the one used on the real HummingBird)
		
		// Controller parameters
		ParamT<bool> *enablePropellers;      	//! Enable/Disable propellers
		ParamT<bool> *enableAttitudeControl; 	//! Enable/Disable attitude control
		ParamT<bool> *enablePositionControl; 	//! Enable/Disable position control
		ParamT<bool> *enableAbsoluteError;   	//! Enable/Disable use of internal absolute position/attitude data
		ParamT<bool> *matlabMode;            	//! Enable/Disable matlabMode (check, maybe it becomes obsolete)
		ParamT<bool> *recordLog;             	//! Enable/Disable data logging (position, orientation, des_position)
		
		// Conversion matrix and vectors for propeller simulation
		arma::Mat<double>::fixed<4,4> torques2omega;   	//! Matrix to convert from torques to propellers rotational speeds
		arma::Col<double>::fixed<4> torques;           	//! Vector of torques (to be converted with torques2omega)
		arma::Col<double>::fixed<4> omega_square;      	//! Vector of propellers rotational speeds
		
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
		void normalize_angle(Vector3& v);
		
		// Input callbacks
		void positionInputCallback(const geometry_msgs::Twist::ConstPtr & msg);
		void torquesInputCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
		void actualInputCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
		void serialInputCallback(const std_msgs::String::ConstPtr& msg);

	public:
		HummingBirdController(Entity *parent);
		~HummingBirdController();
		void LoadChild(XMLConfigNode *node);
		void InitChild();
		void UpdateChild();
		void FiniChild();
};

} // close namespace

#endif
