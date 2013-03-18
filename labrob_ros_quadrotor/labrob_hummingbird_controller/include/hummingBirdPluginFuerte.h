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
    ros::Publisher speechPub;

	// The ARTag estimated position.
	double arX,arY,arZ;

	// An "accumulator value". It is used only in text output.
	double accX,accY,accZ;

	// The velocity value computed by the IMU.
	double imuX, imuY, imuZ;

	// Tag position for X and Y coordinate. We assume that the Z value is alway zero.
	float tagX[8];
	float tagY[8];

	// Time elapsed since the simulation began.
	int timer

	// Time marker for the "switch fase" start.
	int timerSwitch;

	// The current active tag.
	int arIndex;

	// Switch Flags.
	//
	// switchFlag is true when the user calls for a tag switching.
	// arFlag is true if we have an updated estimation of the tags position AFTER the user calls for a tag switching.
	//
	// Due to that a "switch phase" begins if and only if switchFalg AND arFlag are true.
	//
	// switchFlag | arTag | EFFECT                   | CAUSE
	// false      | false | Current target hovering  | Switch ends. Simulation starts.
	// true       | false | Nothing                  | User calls for a tag switching.
	// true       | true  | The switch begins!       | switchFlag is true AND the arTagCallback is called by ARTAG.
	// false      | true  | INVALID STATE!
	//
	bool arFlag, switchFlag
		
	// This flags are used for the commands: "go to the last tag" and "go to the first tag".
	bool beginFlag, endFlag;

	// Output filename
	std::ofstream fileName;

	// The number of tags in the simulation.
    int artagNumber;

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

		/*!
		 * Routine called in the update loop during the "switch phase".
		 */
		void tagSwitching();
		
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

		//Auxiliary function
		float gaussianError();

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
