
#include <hummingBirdPluginFuerte.h>
#include <ar_pose/ARMarkers.h>
#include <stdlib.h>
#include <math.h>

using std::cout;
using std::endl;

#define MY_ZERO 0.000001
#define GRAV 9.81
#define MANDATORY 1
#define OPTIONAL  0

#define M_2PI 2 * M_PI
using gazebo::physics::Link;
using gazebo::physics::JointPtr;
using gazebo::physics::LinkPtr;

namespace gazebo{
enum input_types {POSITION, TORQUES, ACTUAL, SERIAL};

// Constructor
HummingBirdPlugin::HummingBirdPlugin(){
	
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
HummingBirdPlugin::~HummingBirdPlugin() {
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void HummingBirdPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
	// Get and check the parent model
	srand(time(NULL));
	parent_model = _parent;
	world        = _parent->GetWorld();
	arFlag = false;
	switchFlag = false;
	timer=0;
	arIndex=0;
	tagX={0,0,0,0,0,0.5,0.5,1};
	tagY={0,0.5,1,1.5,2,2,2.5,2.5};
	if (!parent_model) gzthrow("Maybe all the controllers requires a Model as their parent");
	
	// Initialize references
	roll_ref   = 0;
	pitch_ref  = 0;
	yaw_ref    = 0;
	thrust_ref = 0;
	
	// Initialize PID gains sets: old gains for ROS electric
// 	p_gain[0] = 1.0;
// 	d_gain[0] = 0.25;
// 	p_gain[1] = 5.0;
// 	d_gain[1] = 8.0;
// 	p_gain[2] = 25.0;
// 	d_gain[2] = 10.0;
	
	// New gains for ROS fuerte
	// [0] -> attitude control
	// [1] -> position control
	// [2] -> thrust roll pitch and yaw torque
	p_gain[0] = 1.0;
	d_gain[0] = 0.25;
	p_gain[1] = 5.0;
	d_gain[1] = 8.0;
	p_gain[2] = 25.0;
	d_gain[2] = 10.0;
	
	
// 	current_position.x = 0;
// 	current_position.y = 0;
// 	current_position.z = 0;
//	desired_position.x = parent_model->GetLink()->GetWorldPose().pos.x;
//	desired_position.y = parent_model->GetLink()->GetWorldPose().pos.y;
//	desired_position.z = parent_model->GetLink()->GetWorldPose().pos.z;

	desired_position.x = 0;
	desired_position.y = 0;
	desired_position.z = 2;

	GPS_position.x = desired_position.x;
	GPS_position.y = desired_position.y;
	GPS_position.z = desired_position.z;
	
	// Allocate variables for propeller management
	joints      = new JointPtr[4];
	body_motors = new LinkPtr[4];
	total_mass = 0;
	
	if (_sdf->HasElement("matlabSimulationMode")) {
		matlabMode = (bool)(_sdf->GetElement("matlabSimulationMode")->GetValueDouble());
	} else {
		matlabMode = false;
	}
	
	robotNamespace = "";
	if (_sdf->HasElement("robotNamespace")){
		robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
	}
	
	// Blades parameters
	if (_sdf->HasElement("motorSpeed")) {
		motorMaxSpeed = _sdf->GetElement("motorSpeed")->GetValueDouble();
	} else {
		motorMaxSpeed = 1000.0;
	}
	
	if (_sdf->HasElement("thrustScale")) {
		thrustScale = _sdf->GetElement("thrustScale")->GetValueDouble();
	} else {
		thrustScale = 350.0;
	}
	
	// Physical / control selection
	if (_sdf->HasElement("enablePropellers")) {
		enablePropellers = (bool)(_sdf->GetElement("enablePropellers")->GetValueDouble());
	} else {
		enablePropellers = false;
	}
	
	if (_sdf->HasElement("enableAttitudeControl")) {
		enableAttitudeControl =(bool)(_sdf->GetElement("enableAttitudeControl")->GetValueDouble());
	} else {
		enableAttitudeControl = false;
	}
	
	if (_sdf->HasElement("enablePositionControl")) {
		enablePositionControl = (bool)(_sdf->GetElement("enablePositionControl")->GetValueDouble());
	} else {
		enablePositionControl = false;
	}
	
	if (_sdf->HasElement("enableAbsoluteError")) {
		enableAbsoluteError = (bool)(_sdf->GetElement("enableAbsoluteError")->GetValueDouble());
	} else {
		enableAbsoluteError = false;
	}
	
	if (_sdf->HasElement("selectInput")) {
		inputType = _sdf->GetElement("selectInput")->GetValueString();
	} else {
		inputType = "actual";
	}

	if (_sdf->HasElement("recordLog")) {
		recordLog = (bool)(_sdf->GetElement("recordLog")->GetValueDouble());
	} else {
		recordLog = false;
	}

    if (_sdf->HasElement("artagNumber")) {
        artagNumber = _sdf->GetElement("artagNumber")->GetValueInt();
    } else {
        artagNumber = 3;
    }
	
	// Update rate
	if (!_sdf->HasElement("updateRate"))
		update_rate = 100.0;
	else
		update_rate = _sdf->GetElement("updateRate")->GetValueDouble();
	
	// prepare to throttle this plugin at the same rate
	// ideally, we should invoke a plugin update when the sensor updates,
	// have to think about how to do that properly later
	if (update_rate > 0.0)
		update_period = 1.0/update_rate;
	else
		update_period = 0.0;
	
	last_update_time = common::Time(0);
	
	// Get aerodynamic/physical parameters
	// l = length of quadrotor arm
	// b = thrust factor
	// d = drag factor
	// They are used to build the conversion matrix from torques to omega
	double l,b,d;
	if (_sdf->HasElement("armLength"))       { l = _sdf->GetElement("armLength")->GetValueDouble();       }
	else { l = 0.16; }
	
	if (_sdf->HasElement("propellerThrust")) { b = _sdf->GetElement("propellerThrust")->GetValueDouble(); }
	else { b = 3.5e-5; }
	
	if (_sdf->HasElement("propellerDrag"))   { d = _sdf->GetElement("propellerDrag")->GetValueDouble();   }
	else { d = 7.5e-7; }
	
	propDragCoeff   = d;
	propThrustCoeff = b;
	
	cout << "Coefficients (b,d,l): { " << b << ", "<< d << ", "<< l << " }" << endl;
	
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
	
	
	body_main = this->parent_model->GetLink("base_link");
	int childrens_num = this->parent_model->GetChildCount();
	
	cout << "Tree: " << childrens_num << endl;
	for (int i = 0; i < childrens_num; i++){
		total_mass += parent_model->GetLink(parent_model->GetChild(i)->GetName())->GetInertial()->GetMass();
		cout << "j" << i  << ": " << parent_model->GetChild(i)->GetName() << endl;
		cout << "mass: " << parent_model->GetLink(parent_model->GetChild(i)->GetName())->GetInertial()->GetMass() << endl;
	}
	
	body_motors[0] = parent_model->GetLink("motor1");
	body_motors[1] = parent_model->GetLink("motor2");
	body_motors[2] = parent_model->GetLink("motor3");
	body_motors[3] = parent_model->GetLink("motor4");
	
	// TODO: check joint existence
	childrens_num = parent_model->GetJointCount();
	cout << "TreeJoint: " << childrens_num << endl;
	for (int i = 0; i < childrens_num; i++){
		cout << "j" << i  << ": " << parent_model->GetJoint(i)->GetName() << endl;
	}
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
	topicName += inputType;
	
	selected_input = -1;
	
	// Check validity of control type
	if (inputType.compare("position") == 0) selected_input = POSITION;
	if (inputType.compare("actual"  ) == 0) selected_input = ACTUAL;
	if (inputType.compare("torques" ) == 0) selected_input = TORQUES;
	if (inputType.compare("serial"  ) == 0) selected_input = SERIAL;

  artagSubscriber = rosNodeHandle.subscribe("ar_pose_marker", 5, &HummingBirdPlugin::artagCallback, this);
  imuSubscriber = rosNodeHandle.subscribe("/quad0/imu_vel", 5, &HummingBirdPlugin::imuCallback, this);
  switchSubscriber = rosNodeHandle.subscribe("/ar_marker_switch", 5, &HummingBirdPlugin::switchCallback, this);
  fileName.open("markerLog.txt");

	
	if (selected_input != -1) {
		switch(selected_input) {
			case POSITION:
				inputSubscriber = rosNodeHandle.subscribe(topicName, 5, &HummingBirdPlugin::positionInputCallback, this);
			break;
			case ACTUAL:
				inputSubscriber = rosNodeHandle.subscribe(topicName, 5, &HummingBirdPlugin::actualInputCallback, this);
			break;
			case TORQUES:
				inputSubscriber = rosNodeHandle.subscribe(topicName, 5, &HummingBirdPlugin::torquesInputCallback, this);
			break;
			case SERIAL:
				inputSubscriber = rosNodeHandle.subscribe(topicName, 5, &HummingBirdPlugin::serialInputCallback, this);
			break;
		}
	}
	else {
		gzthrow(endl << "CONTROLLER ERROR: CONTROL TYPE IS INVALID! (you selected " << inputType << ")");
	}
	open_log_file();
	
	// listen to the update event (broadcast every simulation iteration)
  this->updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&HummingBirdPlugin::UpdateChild, this));
}

// Initialize the controller
void HummingBirdPlugin::Init() {}

// Update the controller
void HummingBirdPlugin::UpdateChild() {
	common::Time cur_time = world->GetSimTime();
	if (cur_time - last_update_time >= update_period){
		last_update_time = cur_time;
		if(arFlag && switchFlag){
			double s = (timer-timerSwitch)/10000.0;
			if (s>1){
				arFlag = false;
				switchFlag = false;
				desired_position.z= 2;
				desired_position.x = 0;
				desired_position.y=0;
			}else{
				math::Vector3 tmp = linearInterpol(s);
				desired_position.z=tmp.z;
			       	desired_position.x=tmp.x;
			       	desired_position.y=tmp.y;
			}
		}
		//cout<<desired_position.x<<endl;
		update_ground_truth();
		HighLevelLoop();
		LowLevelLoop();
		PublishData();
		if(recordLog) record_log_file();
	}
}

math::Vector3 HummingBirdPlugin::linearInterpol(double s){
	math::Vector3 result;
	result.x = 0*s+(1-s)*initial_position.x;
	result.y = 0*s+(1-s)*initial_position.y;
	result.z = 2*s+(1-s)*initial_position.z;
	return result;

}

void HummingBirdPlugin::HighLevelLoop() {
	// recover data from nodes
	receive_commands();
	// emulate sonar data
	update_sonar();

	// perform filtering

	// perform control loop
	height_Control_gravity_comp();
	if(enablePositionControl){ position_control();}
}

void HummingBirdPlugin::LowLevelLoop() {
	// Recover all data from sensors/world  
	update_barometer();

	// Perform inner control loop
	// attitude control
	if(enableAttitudeControl) attitude_control();

	// set angular speed for each propeller
	set_motor_speed();

	// convert to HB data
	IMU_update_angles();
	IMU_update_accelerations();
}

void HummingBirdPlugin::receive_commands(){
	ros::spinOnce();
}

void HummingBirdPlugin::IMU_update_angles(){
	roll  = current_rotation_euler.x;
	pitch = current_rotation_euler.y;
	yaw   = current_rotation_euler.z;
};

void HummingBirdPlugin::attitude_control() {
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
			
			if(0 && enablePropellers) {
				cout << "attitude errors  { " << p_error.x   << ", " << p_error.y    << ", " << p_error.z  << " }" << endl;
				cout << "attitude vels    { " << v_error.x   << ", " << v_error.y    << ", " << v_error.z  << " }" << endl;
				cout << "attitude torques { " << roll_torque << ", " << pitch_torque << ", " << yaw_torque << " }" << endl;
			}
	}
}

void HummingBirdPlugin::height_Control_gravity_comp() {

	timer++;
	// p_gain[2] is equal to 25.0
	// d_gain[2] is equal to 10.0
	
	
	//if (1){
	if (timer<5000){
    	//cout<<"Z: "<<arZ-current_position.z<<endl;  
	  	Thrust = p_gain[2]  * (desired_position.z - current_position.z) +
		         d_gain[2]  * (0 - current_velocity.z) +
		         total_mass * GRAV;
	}else{
		//cout<<"5000"<<endl;
		Thrust = p_gain[2]  * 0.5*(desired_position.z - arZ) + 
		         d_gain[2]  * (0 - imuZ) +
		         total_mass * GRAV;
	}
  
  //float sinz = sin(count)+2;
  //count+=0.001;
	//Thrust = p_gain[2]  * (sinz - current_position.z) +
	//         d_gain[2]  * (0 - current_velocity.z) +
	//         total_mass * GRAV;
	if (Thrust < 1.0) Thrust = 1.0;
	if(matlabMode) { Thrust = thrust_ref; }
	
// 	cout << "Thrust: " << Thrust << endl;
	// Vector3 *tmp_force  = new Vector3(0, 0, Thrust);
	// body_main->SetForce(*tmp_force);
}

void HummingBirdPlugin::position_control() {
//TODO: - set feedforward

	//cout<<"X: "<<arX-desired_position.x<<endl;
	//cout<<"Y: "<<arY-desired_position.y<<endl;
	// p_gain[1] is equal to 5.0
	// d_gain[1] is equal to 8.0
	
//	double y_ref = 0.0;
	double y_dot_ref = 0.0;
//	double y_ff = 0.0;
//	double x_ref = 0.0;
	double x_dot_ref = 0.0;
//	double x_ff = 0.0;
	
// 	cout << "pos: [" << current_position.x << " " << current_position.y << " " << current_position.z << "]  des: [" << desired_position.x << " " << desired_position.y << " " << desired_position.z << endl;
	
	double e_x     = 0;
  	double e_x_dot = 0;
  	double e_y     = 0;
  	double e_y_dot = 0;

	//if (1){
	if (timer<5000){
  		e_x     = desired_position.x - current_position.x;
  		e_x_dot = x_dot_ref - current_velocity.x;
  		e_y     = desired_position.y - current_position.y;
  		e_y_dot = y_dot_ref - current_velocity.y;
  	}else{
  		e_x     = 0.5*(desired_position.x - arX);
  		e_x_dot = x_dot_ref - imuX;
  		e_y     = 0.5*(desired_position.y - arY);
  		e_y_dot = y_dot_ref - imuY;
		if (1){
			fileName << arX+tagX[arIndex] << ";" << arY+tagY[arIndex] << ";" << arZ << ";" << current_position.x << ";" << current_position.y << ";" <<current_position.z << ";" << desired_position.x+tagX[arIndex] << ";" << desired_position.y+tagY[arIndex] << ";" << desired_position.z << endl;
		}
  	}
	
// 	cout << "errX: " << e_x << " errY: " << e_y << endl;
	
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
	
// 	cout << "rollRef: " << roll_ref*180.0/M_PI << " pitchRef: " << pitch_ref*180.0/M_PI << endl;
// 	cout << "roll: " << roll*180.0/M_PI << " pitch: " << pitch*180.0/M_PI << endl;
}

void HummingBirdPlugin::set_motor_speed() {
	// NOTE: propelellers as well as motors and arms are in clockwise order
	//			0
	//		3 	1
	//			2

if (Thrust < 0) Thrust = 0;
	if(enablePropellers) {
		// See constructor for the matrix torques2omega
		torques << roll_torque << pitch_torque << yaw_torque << Thrust;
		omega_square = torques2omega * torques;
		
// 		cout << "rollT: " << roll_torque << " pitchT: " << pitch_torque << " yawT: " << yaw_torque << " thrust: " << Thrust << endl;
		
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
		motor_force[0].z = pow(joints[0]->GetVelocity(2),2) * propThrustCoeff;
		motor_force[1].z = pow(joints[1]->GetVelocity(2),2) * propThrustCoeff;
		motor_force[2].z = pow(joints[2]->GetVelocity(2),2) * propThrustCoeff;
		motor_force[3].z = pow(joints[3]->GetVelocity(2),2) * propThrustCoeff;

		// Set yaw torque
// TODO: I WOULD LIKE (19/04/2012) TO CONSIDER THE ACTUAL ANGULAR VELOCITY
		motor_force[0].x = prop_speed[0] * propDragCoeff;
		motor_force[1].x = prop_speed[1] * propDragCoeff;
		motor_force[2].x = prop_speed[2] * propDragCoeff;
		motor_force[3].x = prop_speed[3] * propDragCoeff;
		
		motor_force[0].y = 0.0;
		motor_force[1].y = 0.0;
		motor_force[2].y = 0.0;
		motor_force[3].y = 0.0;
		
		// Set resulting forces
		// TODO: NO! (19/04/2012) NOW I THINK THIS IS IN WORLD FRAME (TO BE CHECKED)
		// THEN WE NEED TO ROTATE THESE FORCES IN ORDER TO SET THEM IN EACH
		// MOTOR RELATIVE FRAME
		
		// NOTE: IN ROS FUERTE THE SET FORCE METHOD IS IN WORLD COORDINATE
		
// 		cout << "roll: " << roll*180/M_PI << " pitch: " << pitch*180/M_PI << " yaw: " << yaw*180/M_PI << endl;
		// Rotate the force in absolute frame axes
		abs2rel(0,0)   = cos(pitch)*cos(yaw);
		abs2rel(0,1)   = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
		abs2rel(0,2)   = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
		abs2rel(1,0)   = cos(pitch)*sin(yaw);
		abs2rel(1,1)   = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
		abs2rel(1,2)   = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
		abs2rel(2,0)   = -sin(pitch);
		abs2rel(2,1)   = cos(pitch)*sin(roll);
		abs2rel(2,2)   = cos(pitch)*cos(roll);
		
// 		cout << "rotMatrix: " << abs2rel << endl;
// 		cout << "motForce: " << motor_force[0] << " " << motor_force[1] << " " << motor_force[2] << " " << motor_force[3] << endl;
		
		for(int i=0;i<4;++i){
			mot2trasf(0,i) = motor_force[i].x;
			mot2trasf(1,i) = motor_force[i].y;
			mot2trasf(2,i) = motor_force[i].z;
		}
		
		mot2trasf = (abs2rel) * mot2trasf;
		
		for(int i=0;i<4;++i){
			(motor_force[i]).Set(mot2trasf(0,i), mot2trasf(1,i), mot2trasf(2,i));
		}
		
// 		cout << "forces: " << motor_force[0] << " " << motor_force[1] << " " << motor_force[2] << " " << motor_force[3] << endl;
		body_motors[0]->SetForce(motor_force[0]);
		body_motors[1]->SetForce(motor_force[1]);
		body_motors[2]->SetForce(motor_force[2]);
		body_motors[3]->SetForce(motor_force[3]);
	}
	else {
		
		// Rotate the force in absolute frame axes
		abs2rel(0,0)   = cos(pitch)*cos(yaw);
		abs2rel(0,1)   = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
		abs2rel(0,2)   = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
		abs2rel(1,0)   = cos(pitch)*sin(yaw);
		abs2rel(1,1)   = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
		abs2rel(1,2)   = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
		abs2rel(2,0)   = -sin(pitch);
		abs2rel(2,1)   = cos(pitch)*sin(roll);
		abs2rel(2,2)   = cos(pitch)*cos(roll);
		
		tmpVec(0) = roll_torque; 
		tmpVec(1) = pitch_torque; 
		tmpVec(2) = yaw_torque; 
		
		tmpVec = abs2rel * tmpVec;
		
		// Torque in world frame (check if needed)
		body_main->SetTorque(math::Vector3(tmpVec(0), tmpVec(1), tmpVec(2)));
		
		// Force in world reference frame
		tmpVec(0) = 0.0; 
		tmpVec(1) = 0.0; 
		tmpVec(2) = Thrust; 
		
		tmpVec = abs2rel * tmpVec;
		
		body_main->SetForce(math::Vector3(tmpVec(0), tmpVec(1), tmpVec(2)));
		
	}
}

// 	
// 	/***************************************************************/
// 	/*                                                             */
// 	/*  this is called at every update simulation step             */
// 	/*                                                             */
// 	/***************************************************************/

void HummingBirdPlugin::artagCallback(const ar_pose::ARMarkers::ConstPtr& msg) {
	  
		math::Vector3 tmp;
		if(markerSearch(arIndex, msg, tmp)){
	  	float diffX = pow(arX- tmp.x,2);//msg->markers[arIndex].pose.pose.position.y,2);
	  	float diffY = pow(arY- tmp.y,2);//msg->markers[arIndex].pose.pose.position.x,2);
	  	float diffZ = pow(arZ- tmp.z,2);//msg->markers[arIndex].pose.pose.position.z,2);

	  //arX=msg->markers[arIndex].pose.pose.position.y*exp(-diffX)+arX*(1-exp(-diffX));
	  //arY=msg->markers[arIndex].pose.pose.position.x*exp(-diffY)+arY*(1-exp(-diffY));
	  //arZ=msg->markers[arIndex].pose.pose.position.z*exp(-diffZ)+arZ*(1-exp(-diffZ));
		arX=tmp.x*exp(-diffX)+arX*(1-exp(-diffX));
	  	arY=tmp.y*exp(-diffY)+arY*(1-exp(-diffY));
	  	arZ=tmp.z*exp(-diffZ)+arZ*(1-exp(-diffZ));
		}

	 //Naive smoothing
	 //
	 // if((diffX < 0.2 && diffY < 0.2 && diffZ < 0.2) || timer < 5000 || smoothFlag){
	 //       
	 // 	arX=msg->markers[arIndex].pose.pose.position.y ;
         // 	arY=msg->markers[arIndex].pose.pose.position.x;
         // 	arZ=msg->markers[arIndex].pose.pose.position.z;
	 //       if (smoothFlag) smoothFlag = false;
	 //       	
	 // }

	  if(switchFlag && arIndex<8 && !arFlag){
		cout<<arIndex<<endl;
		cout<<switchFlag<<endl;
		//initial_position.x=msg->markers[arIndex+1].pose.pose.position.y;
		//initial_position.y=msg->markers[arIndex+1].pose.pose.position.x;
		//initial_position.z=msg->markers[arIndex+1].pose.pose.position.z;
		math::Vector3 tmp2;
		if(markerSearch(arIndex, msg, tmp2)){
			initial_position.x=tmp2.x;
			initial_position.y=tmp2.y;
			initial_position.z=tmp2.z;
			//accX += -initial_position.x;
			//accY += -initial_position.y;
			//accZ += initial_position.z;
			arFlag = true;
		}
	  }

}

void HummingBirdPlugin::switchCallback(const std_msgs::String::ConstPtr& msg) {
	if(msg->data=="FORWARD" && arIndex<8){
		arIndex++;
		switchFlag = true;
		smoothFlag = true;
		timerSwitch=timer;
	}else if(msg->data=="BACKWARD" && arIndex>0){
		arIndex--;
		switchFlag = true;
		smoothFlag = true;
		timerSwitch=timer;
	}
}

void HummingBirdPlugin::imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
	imuX=msg->vector.x;
	imuY=msg->vector.y;
	imuZ=msg->vector.z;
	
	/*cout<<msg->vector.x<<endl;
	cout<<msg->vector.y<<endl;
	cout<<msg->vector.z<<endl;
	cout<<"--"<<endl;*/



}

void HummingBirdPlugin::positionInputCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	// TODO: fix this it is ugly!! (divide ground truth from GPS!!)
	if (! enableAbsoluteError) {
		current_position.x = msg->linear.x;
		current_position.y = msg->linear.y;
		current_position.z = msg->linear.z;
	}
	desired_position.x = msg->angular.x;
	desired_position.y = msg->angular.y;
	desired_position.z = msg->angular.z;
}

void HummingBirdPlugin::actualInputCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
	roll_ref   = msg->x;
	pitch_ref  = msg->y;
	yaw_ref    = msg->z;
	thrust_ref = msg->w;
}

void HummingBirdPlugin::torquesInputCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
	roll_torque  = msg.get()->x;
	pitch_torque = msg.get()->y;
	yaw_torque   = msg.get()->z;
	Thrust       = msg.get()->w;
}

void HummingBirdPlugin::serialInputCallback(const std_msgs::String::ConstPtr& msg) {
	// TODO: we need to subdivide the hummingBird code in modules,
	//       in order to use the submodules in here
	cout << "STRING MESSAGE RECEIVED: " << msg.get() << endl;
}

void HummingBirdPlugin::record_log_file() {
	logFile << std::fixed << std::showpos << ros::Time::now().toSec()  << " "
	<< GPS_position.x           << " " << GPS_position.y           << " " << GPS_position.z           << " "
	<< current_rotation_euler.x << " " << current_rotation_euler.y << " " << current_rotation_euler.z << " "
	<< current_velocity.x       << " " << current_velocity.y       << " " << current_velocity.z       << " "
	<< current_rotation_vel.x   << " " << current_rotation_vel.y   << " " << current_rotation_vel.z   << " "
	<< current_position.x       << " " << current_position.y       << " " << current_position.z       << " "
	<< desired_position.x       << " " << desired_position.y       << " " << desired_position.z       << " "
	<< endl;
}

void HummingBirdPlugin::open_log_file() {
	logFile.open(parent_model->GetName().append("_log.dat").c_str());
	logFile << "Recorded from Gazebo controller, model name " << parent_model->GetName() << endl
	        << "time x y z roll pitch yaw vx vy vz p q r xest yest zest xdes ydes zdes"
	        << endl;
}

void HummingBirdPlugin::close_log_file() {
	logFile.close();
}

void HummingBirdPlugin::update_ground_truth() {
	GPS_position           = body_main->GetWorldPose().pos;
	current_position       = body_main->GetWorldPose().pos;
	current_rotation_euler = body_main->GetWorldPose().rot.GetAsEuler();
	current_velocity       = body_main->GetWorldLinearVel();
	current_rotation_vel   = body_main->GetRelativeAngularVel();
}


// Auxiliar functions

float HummingBirdPlugin::gaussianError(){	
	float i,j;
	i=(rand()+1.0)/RAND_MAX;
	j=(rand()+1.0)/RAND_MAX;
	
	

	float error = sqrt(-2*log(i))*cos(2*M_PI*j);
	
	cout << i << ";" << j << ";" << error << endl;
	return error;
}

double HummingBirdPlugin::zero_check(double v){
	if(v > -MY_ZERO && v < MY_ZERO) return 0.0;
	else return v;
}

bool HummingBirdPlugin::markerSearch(int id, const ar_pose::ARMarkers::ConstPtr& msg, math::Vector3& markerPose){
	for(int i=0; i<msg->markers.size(); i++){
		if(id==msg->markers[i].id){
			markerPose.x = msg->markers[i].pose.pose.position.y+0.05*gaussianError();
			markerPose.y = msg->markers[i].pose.pose.position.x+0.05*gaussianError();
			markerPose.z = msg->markers[i].pose.pose.position.z+0.05*gaussianError();
			return true;
		}
	}
	return false;
}

void HummingBirdPlugin::normalize_angle(double& a){
	// DO WE NEED THIS? CHECK!
}

void HummingBirdPlugin::normalize_angle(math::Vector3& v){
	// DO WE NEED THIS? CHECK!
}

void HummingBirdPlugin::FiniChild(){
}

GZ_REGISTER_MODEL_PLUGIN(HummingBirdPlugin);
}
