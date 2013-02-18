
#include <algorithm>
#include <assert.h>

#include <hummingBirdController.h>

#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/Body.hh>

#include <iostream>

#include <ros/ros.h>

using namespace gazebo;
using std::cout;
using std::endl;

#define MY_ZERO 0.000001
#define GRAV 9.81
#define MANDATORY 1
#define OPTIONAL  0

#define M_2PI 2 * M_PI

GZ_REGISTER_DYNAMIC_CONTROLLER("labrob_hubi_controller", HummingBirdController);

enum input_types {POSITION, TORQUES, ACTUAL, SERIAL};

// Constructor
HummingBirdController::HummingBirdController(Entity *parent) : Controller(parent) {
	// Get and check the parent model
	parent_model = dynamic_cast<Model*> (parent);
	if (!parent_model) gzthrow("Maybe all the controllers requires a Model as their parent");
	
	// Initialize references
	roll_ref   = 0;
	pitch_ref  = 0;
	yaw_ref    = 0;
	thrust_ref = 0;
	
	// Initialize PID gains sets
	p_gain[0] = 1.0;
	d_gain[0] = 0.25;
	p_gain[1] = 5.0;
	d_gain[1] = 8.0;
	p_gain[2] = 25.0;
	d_gain[2] = 10.0;
	
	
// 	current_position.x = 0;
// 	current_position.y = 0;
// 	current_position.z = 0;
	desired_position.x = parent_model->GetBody()->GetWorldPose().pos.x;
	desired_position.y = parent_model->GetBody()->GetWorldPose().pos.y;
	desired_position.z = parent_model->GetBody()->GetWorldPose().pos.z;

	GPS_position.x = desired_position.x;
	GPS_position.y = desired_position.y;
	GPS_position.z = desired_position.z;
	
	// Allocate variables for propeller management
	joints      = new Joint*[4];
	body_motors = new Body*[4];
	total_mass = 0;
	
	// Parse parameters (taken from gazebo_ros_camera plugin)
	Param::Begin(&this->parameters);
	armLength       = new ParamT<double>("armLength", 0.16, MANDATORY);
	matlabMode      = new ParamT<bool>("matlabSimulationMode",false  , OPTIONAL);
	robotNamespaceP = new ParamT<std::string>("robotNamespace","/"   , OPTIONAL);
	
	// Blades parameters
	propDragCoeff 	= new ParamT<double>("propellerDrag"  , 7.5e-7, MANDATORY);
	propThrustCoeff = new ParamT<double>("propellerThrust", 3.5e-5, MANDATORY);
	motorMaxSpeed 	= new ParamT<double>("motorSpeed"			, 1000.0, MANDATORY);
	thrustScale 		= new ParamT<double>("thrustScale"		, 350.0 , MANDATORY);
	
	// Physical / control selection
	enablePropellers      = new ParamT<bool>("enablePropellers"     , true, OPTIONAL);
	enableAttitudeControl = new ParamT<bool>("enableAttitudeControl", true, OPTIONAL);
	enablePositionControl = new ParamT<bool>("enablePositionControl", true, OPTIONAL);
	enableAbsoluteError   = new ParamT<bool>("enableAbsoluteError"  , true, OPTIONAL);
	inputType = new ParamT<std::string>("selectInput", "actual", MANDATORY);
	
	// Additional parameters
	recordLog = new ParamT<bool>("recordLog"            , false,OPTIONAL);
	Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
HummingBirdController::~HummingBirdController() {
	close_log_file();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void HummingBirdController::LoadChild(XMLConfigNode *node) {
	body_main = this->parent_model->GetBody("base_link");
	const std::vector<Entity*> childrens = this->parent_model->GetChildren();
	
	// TODO: add and manage the other links
	int size = childrens.size();
	cout << "Tree: " << size << endl;
	for (int i = 0; i < size; i++){
		cout << "j" << i  << ": " << childrens[i]->GetName() << endl;
		total_mass += parent_model->GetBody(childrens[i]->GetName())->GetMass().GetAsDouble();
	}
	
	// Load options from URDF file
	armLength->Load(node);
	propDragCoeff->Load(node);
	propThrustCoeff->Load(node);
	motorMaxSpeed->Load(node);
	thrustScale->Load(node);
	enableAttitudeControl->Load(node);
	enablePositionControl->Load(node);
	enableAbsoluteError->Load(node);
	enablePropellers->Load(node);
	recordLog->Load(node);
	inputType->Load(node);
	matlabMode->Load(node);
	
	body_motors[0] = parent_model->GetBody("motor1");
	body_motors[1] = parent_model->GetBody("motor2");
	body_motors[2] = parent_model->GetBody("motor3");
	body_motors[3] = parent_model->GetBody("motor4");
	
	// TODO: check joint existence
	joints[0] = parent_model->GetJoint("arm_motor1"); // motor up
	joints[1] = parent_model->GetJoint("arm_motor2"); // motor right
	joints[2] = parent_model->GetJoint("arm_motor3"); // motor down
	joints[3] = parent_model->GetJoint("arm_motor4"); // motor left
	
	// TODO: check proper value
	double maxMotorForce = 100.0;
	joints[0]->SetMaxForce(2, maxMotorForce);
	joints[1]->SetMaxForce(2, maxMotorForce);
	joints[2]->SetMaxForce(2, maxMotorForce);
	joints[3]->SetMaxForce(2, maxMotorForce);
	
	// Build UNIQUE topic name
	std::string topicName = "/";
	topicName += parent_model->GetName();
	topicName += "/input/";
	topicName += inputType->GetAsString();
	
	selected_input = -1;
	
	// Check validity of control type
	if (inputType->GetAsString().compare("position") == 0) selected_input = POSITION;
	if (inputType->GetAsString().compare("actual"  ) == 0) selected_input = ACTUAL;
	if (inputType->GetAsString().compare("torques" ) == 0) selected_input = TORQUES;
	if (inputType->GetAsString().compare("serial"  ) == 0) selected_input = SERIAL;
	
	if (selected_input != -1) {
		switch(selected_input) {
			case POSITION:
				inputSubscriber = rosNodeHandle.subscribe(topicName, 5, &HummingBirdController::positionInputCallback, this);
			break;
			case ACTUAL:
				inputSubscriber = rosNodeHandle.subscribe(topicName, 5, &HummingBirdController::actualInputCallback, this);
			break;
			case TORQUES:
				inputSubscriber = rosNodeHandle.subscribe(topicName, 5, &HummingBirdController::torquesInputCallback, this);
			break;
			case SERIAL:
				inputSubscriber = rosNodeHandle.subscribe(topicName, 5, &HummingBirdController::serialInputCallback, this);
			break;
		}
	}
	else {
		gzthrow(endl << "CONTROLLER ERROR: CONTROL TYPE IS INVALID! (you selected " << inputType->GetAsString() << ")");
	}
	// Get aerodynamic/physical parameters
	double l = armLength->GetValue();
	double b = propThrustCoeff->GetValue();
	double d = propDragCoeff->GetValue();
	
	cout << "coefficients (b,d,l): { " << b << ", "<< d << ", "<< l << " }" << endl;
	
	// THIS IS IN NWU FRAME!
	// This comes from the inversion of 
	// 	[   0,  b*l,    0, -b*l]
	// 	[-b*l,    0,  b*l,    0]
	// 	[  -d,    d,   -d,    d] = omega2torques
	// 	[   b,    b,    b,    b]
	// note : torques = omega2torques * Omega^2
	//
	torques2omega <<          0 << -1/(2*b*l) << -1/(4*d*l) << 1/(4*b) << arma::endr
	              <<  1/(2*b*l) <<          0 <<  1/(4*d*l) << 1/(4*b) << arma::endr
	              <<          0 <<  1/(2*b*l) << -1/(4*d*l) << 1/(4*b) << arma::endr
	              << -1/(2*b*l) <<          0 <<  1/(4*d*l) << 1/(4*b);

	open_log_file();
}

// Initialize the controller
void HummingBirdController::InitChild() {}

// Update the controller
void HummingBirdController::UpdateChild() {
	update_ground_truth();
	HighLevelLoop();
	LowLevelLoop();
	PublishData();
	if(recordLog->GetValue()) record_log_file();
}

// Finalize the controller
void HummingBirdController::FiniChild() {}

void HummingBirdController::HighLevelLoop() {
	// recover data from nodes
	receive_commands();
	// emulate sonar data
	update_sonar();

	// perform filtering

	// perform control loop
	height_Control_gravity_comp();
	if(enablePositionControl->GetValue()) position_control();
}

void HummingBirdController::LowLevelLoop() {
	// Recover all data from sensors/world  
	update_barometer();

	// Perform inner control loop
	// attitude control
	if(enableAttitudeControl->GetValue()) attitude_control();

	// set angular speed for each propeller
	set_motor_speed();

	// convert to HB data
	IMU_update_angles();
	IMU_update_accelerations();
}

void HummingBirdController::receive_commands(){
  ros::spinOnce();
}

void HummingBirdController::IMU_update_angles(){
	roll  = current_rotation_euler.x;
	pitch = current_rotation_euler.y;
	yaw   = current_rotation_euler.z;
};

void HummingBirdController::attitude_control() {
	// p_gain[2] is equal to 1.0
	// d_gain[2] is equal to 0.25
	
	switch(selected_input) {
		case TORQUES:
		break;
		default:
			// Modified to respect the real robot control mode: yaw input = 0 maintains current yaw
			p_error.x = zero_check(roll_ref  - current_rotation_euler.x);
			p_error.y = zero_check(pitch_ref - current_rotation_euler.y);
			p_error.z = zero_check(yaw_ref   - current_rotation_euler.z);
			
			v_error = current_rotation_vel;
			
			roll_torque   = (p_gain[0] * p_error.x - d_gain[0] * v_error.x);
			pitch_torque  = (p_gain[0] * p_error.y - d_gain[0] * v_error.y);
			yaw_torque    = (p_gain[0] * p_error.z - d_gain[0] * v_error.z); //  yaw_torque = 0.0;
			
			if(0 && enablePropellers->GetValue()) {
				cout << "attitude errors  { " << p_error.x   << ", " << p_error.y    << ", " << p_error.z  << " }" << endl;
				cout << "attitude vels    { " << v_error.x   << ", " << v_error.y    << ", " << v_error.z  << " }" << endl;
				cout << "attitude torques { " << roll_torque << ", " << pitch_torque << ", " << yaw_torque << " }" << endl;
			}
	}
}

void HummingBirdController::height_Control_gravity_comp() {
	// p_gain[2] is equal to 25.0
	// d_gain[2] is equal to 10.0
	
	Thrust = p_gain[2]  * (desired_position.z - current_position.z) +
	         d_gain[2]  * (0 - current_velocity.z) +
	         total_mass * GRAV;
	if (Thrust < 1.0) Thrust = 1.0;
	if(matlabMode->GetValue()) { Thrust = thrust_ref; }
	
	// Vector3 *tmp_force  = new Vector3(0, 0, Thrust);
	// body_main->SetForce(*tmp_force);
}

void HummingBirdController::position_control() {
//TODO: - set feedforward

	// p_gain[1] is equal to 5.0
	// d_gain[1] is equal to 8.0
	
//	double y_ref = 0.0;
	double y_dot_ref = 0.0;
//	double y_ff = 0.0;
//	double x_ref = 0.0;
	double x_dot_ref = 0.0;
//	double x_ff = 0.0;

	double e_x     = desired_position.x - current_position.x;
	double e_x_dot = x_dot_ref - current_velocity.x;
	double e_y     = desired_position.y - current_position.y;
	double e_y_dot = y_dot_ref - current_velocity.y;

	double ex_y     =  cos(yaw) * e_x     + sin(yaw) * e_y;
	double ey_y     = -sin(yaw) * e_x     + cos(yaw) * e_y;
	double ex_y_dot =  cos(yaw) * e_x_dot + sin(yaw) * e_y_dot;
	double ey_y_dot = -sin(yaw) * e_x_dot + cos(yaw) * e_y_dot;
	
	double vc_pitch = (p_gain[1] * ex_y + d_gain[1] * ex_y_dot);
	double vc_roll  = (p_gain[1] * ey_y + d_gain[1] * ey_y_dot);
	
	// Check that it is in the domain of arcsin (maybe it needs a full revision ;) )
	double sat = 0.99;
	if (vc_roll < -sat) vc_roll = -sat;
	if (vc_roll >  sat) vc_roll =  sat;

	if (vc_pitch < -sat) vc_pitch = -sat;
	if (vc_pitch >  sat) vc_pitch =  sat;

	if (Thrust != 0.0) {
		roll_ref  = -asin( (total_mass/Thrust) * vc_roll);
		pitch_ref =  asin( (total_mass/Thrust) * (vc_pitch/cos(roll)) );
	}
  else {
		roll_ref  = 0.0;
		pitch_ref = 0.0;
	}

// 	if (Thrust != 0.0) {
// 		roll_ref = -atan( (1.0/(cos(roll) * cos(yaw))) * ( vir_c * (total_mass/Thrust) - sin(yaw) * sin(pitch) * cos(roll)) );
// 	}
//   else roll_ref = 0.0;

	sat = 0.6;
	if (roll_ref < -sat) roll_ref = -sat;
	if (roll_ref >  sat) roll_ref =  sat;

	if (pitch_ref < -sat) pitch_ref = -sat;
	if (pitch_ref >  sat) pitch_ref =  sat;
}

void HummingBirdController::set_motor_speed() {
	// NOTE: propelellers as well as motors and arms are in clockwise order
	//			0
	//		3 	1
	//			2

if (Thrust < 0) Thrust = 0;
	if(enablePropellers->GetValue()) {
		// See constructor for the matrix torques2omega
		torques << roll_torque << pitch_torque << yaw_torque << Thrust;
		omega_square = torques2omega * torques;
		
		// Calculate propeller speed
		prop_speed[0] =  sqrt(fabs(omega_square[0]));
		prop_speed[1] = -sqrt(fabs(omega_square[1]));
		prop_speed[2] =  sqrt(fabs(omega_square[2]));
		prop_speed[3] = -sqrt(fabs(omega_square[3]));
		
		// Set propeller velocities
		joints[0]->SetVelocity(2, prop_speed[0]);
		joints[1]->SetVelocity(2, prop_speed[1]);
		joints[2]->SetVelocity(2, prop_speed[2]);
		joints[3]->SetVelocity(2, prop_speed[3]);

		// Set Thrust force
//  FROM NOW ON (19/04/2012) WE CONSIDER THE ACTUAL ANGULAR VELOCITIES
		motor_force[0].z = pow(joints[0]->GetVelocity(2),2) * propThrustCoeff->GetValue();
		motor_force[1].z = pow(joints[1]->GetVelocity(2),2) * propThrustCoeff->GetValue();
		motor_force[2].z = pow(joints[2]->GetVelocity(2),2) * propThrustCoeff->GetValue();
		motor_force[3].z = pow(joints[3]->GetVelocity(2),2) * propThrustCoeff->GetValue();

		// Set yaw torque
// TODO: I WOULD LIKE (19/04/2012) TO CONSIDER THE ACTUAL ANGULAR VELOCITY
		motor_force[0].x = prop_speed[0] * propDragCoeff->GetValue();
		motor_force[1].x = prop_speed[1] * propDragCoeff->GetValue();
		motor_force[2].x = prop_speed[2] * propDragCoeff->GetValue();
		motor_force[3].x = prop_speed[3] * propDragCoeff->GetValue();
		
		// Set resulting forces
		// TODO: NO! (19/04/2012) NOW I THINK THIS IS IN WORLD FRAME (TO BE CHECKED)
		// THEN WE NEED TO ROTATE THESE FORCES IN ORDER TO SET THEM IN EACH
		// MOTOR RELATIVE FRAME
		body_motors[0]->SetForce(motor_force[0]);
		body_motors[1]->SetForce(motor_force[1]);
		body_motors[2]->SetForce(motor_force[2]);
		body_motors[3]->SetForce(motor_force[3]);
	}
	else {
		body_main->SetTorque(Vector3(roll_torque, pitch_torque, yaw_torque));
		body_main->SetForce(Vector3(0.0, 0.0, Thrust));
	}
}

// 	
// 	/***************************************************************/
// 	/*                                                             */
// 	/*  this is called at every update simulation step             */
// 	/*                                                             */
// 	/***************************************************************/

void HummingBirdController::positionInputCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	// TODO: fix this it is ugly!! (divide ground truth from GPS!!)
	if (! enableAbsoluteError->GetValue()) {
		current_position.x = msg->linear.x;
		current_position.y = msg->linear.y;
		current_position.z = msg->linear.z;
	}
	desired_position.x = msg->angular.x;
	desired_position.y = msg->angular.y;
	desired_position.z = msg->angular.z;
}

void HummingBirdController::actualInputCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
	roll_ref   = msg->x;
	pitch_ref  = msg->y;
	yaw_ref    = msg->z;
	thrust_ref = msg->w;
}

void HummingBirdController::torquesInputCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
	roll_torque  = msg.get()->x;
	pitch_torque = msg.get()->y;
	yaw_torque   = msg.get()->z;
	Thrust       = msg.get()->w;
}

void HummingBirdController::serialInputCallback(const std_msgs::String::ConstPtr& msg) {
	// TODO: we need to subdivide the hummingBird code in modules,
	//       in order to use the submodules in here
	cout << "STRING MESSAGE RECEIVED: " << msg.get() << endl;
}

void HummingBirdController::record_log_file() {
	logFile << std::fixed << std::showpos << ros::Time::now().toSec()  << " "
	<< GPS_position.x           << " " << GPS_position.y           << " " << GPS_position.z           << " "
	<< current_rotation_euler.x << " " << current_rotation_euler.y << " " << current_rotation_euler.z << " "
	<< current_velocity.x       << " " << current_velocity.y       << " " << current_velocity.z       << " "
	<< current_rotation_vel.x   << " " << current_rotation_vel.y   << " " << current_rotation_vel.z   << " "
	<< current_position.x       << " " << current_position.y       << " " << current_position.z       << " "
	<< desired_position.x       << " " << desired_position.y       << " " << desired_position.z       << " "
	<< endl;
}

void HummingBirdController::open_log_file() {
	logFile.open(parent_model->GetName().append("_log.dat").c_str());
	logFile << "Recorded from Gazebo controller, model name " << parent_model->GetName() << endl
	        << "time x y z roll pitch yaw vx vy vz p q r xest yest zest xdes ydes zdes"
	        << endl;
}

void HummingBirdController::close_log_file() {
	logFile.close();
}

void HummingBirdController::update_ground_truth() {
	GPS_position           = this->body_main->GetWorldPose().pos;
	current_position       = this->body_main->GetWorldPose().pos;
	current_rotation_euler = this->body_main->GetWorldPose().rot.GetAsEuler();
	current_velocity       = this->body_main->GetWorldLinearVel();
	current_rotation_vel   = this->body_main->GetRelativeAngularVel();
}


// Auxiliar functions
double HummingBirdController::zero_check(double v){
	if(v > -MY_ZERO && v < MY_ZERO) return 0.0;
	else return v;
}

void HummingBirdController::normalize_angle(double& a){
	// DO WE NEED THIS? CHECK!
}

void HummingBirdController::normalize_angle(Vector3& v){
	// DO WE NEED THIS? CHECK!
}


