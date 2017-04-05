#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;

// Location of URDF files specifying world and robot information
const string world_file = "resources/hw0/world.urdf";
const string robot_file = "resources/hw0/RRPbot.urdf";
const string robot_name = "RRPbot";

// Redis is just a key value store, publish/subscribe is also possible
// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY  = "cs225a::robot::RRPbot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::robot::RRPbot::sensors::dq";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// Make sure redis-server is running at localhost with default port 6379
	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	/*
	These are mathematical vectors from the library Eigen, you can read up on the documentation online.
	You can input your joint information and read sensor data C++ style "<<" or ">>". Make sure you only 
	expect to read or are writing #D.O.F. number of values.
	*/
	robot->_q << 0.0, 0.0, 0.1; // Joint 1,2,3 Coordinates (radians, radians, meters)
	robot->_dq << 0, 0, 0; // Joint 1,2,3 Velocities (radians/sec, radians/sec, meters/sec), not used here

	/* 
	Here we use our redis set method to serialize an 'Eigen' vector into a specific Redis Key
	Changing set to get populates the 'Eigen' vector given
	This key is then read by the physics integrator or visualizer to update the system
	*/
	redis_client.setEigenMatrixDerivedString(JOINT_ANGLES_KEY,robot->_q);
	redis_client.setEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

	/*
	Update model calculates and updates robot kinematics model information 
	(calculate current jacobian, mass matrix, etc..)
	Values taken from robot-> will be updated to currently set _q values
	*/
	robot->updateModel();

	int dof = robot->dof();

	// operational space
	std::string op_pos_task_link_name = "link2"; // Link of the "Task" or "End Effector"

	// Position of Task Frame in relation to Link Frame (When using custom E.E. attachment, etc..)
	Eigen::Vector3d op_pos_task_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.0); 
	
	Eigen::MatrixXd op_pos_task_jacobian(3,dof); // Empty Jacobian Matrix sized to right size
	
	robot->Jv(op_pos_task_jacobian,op_pos_task_link_name,op_pos_task_pos_in_link); // Read jacobian into op_pos_task_jacobian
	cout << op_pos_task_jacobian << endl; // Print Jacobian
	cout << robot->_M << endl; // Print Mass Matrix, you can index into this variable (and all 'Eigen' types)!

	/* 
	Retrieve multiple values of jacobian or M with a for loop of setting robot->_q's, 
	setting redis keys for display update if needed and don't forget robot->updateModel()! 
	We'll have a logger for you later to dump redis values at whatever rate you choose
	*/

	// This function retrives absolute position of Task Frame in Base Frame
	//Eigen::Vector3d initial_position;
	//robot->position(initial_position,op_pos_task_link_name,op_pos_task_pos_in_link);

	// Wait for user input before quit
	int wait;
	cin >> wait;

    return 0;
}

