// This example application runs a controller for the IIWA

#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;

const string world_file = "resources/hw0/world.urdf";
const string robot_file = "resources/hw0/RRPbot.urdf";
const string robot_name = "RRPbot";

	//Eigen::Vector3d initial_position;
	//robot->position(initial_position,op_pos_task_link_name,op_pos_task_pos_in_link);

// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY  = "scl::robot::iiwaBot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "scl::robot::iiwaBot::sensors::dq";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	// write to Redis
	robot->_q << 0,0,0; // Joint 1,2,3 Values
	robot->_dq << 0,0,0;

	redis_client.setEigenMatrixDerivedString(JOINT_ANGLES_KEY,robot->_q);
	redis_client.setEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	robot->updateModel();

	int dof = robot->dof();

	// operational space position task
	std::string op_pos_task_link_name = "link3";
	Eigen::Vector3d op_pos_task_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.0);
	Eigen::MatrixXd op_pos_task_jacobian(3,dof);
	robot->Jv(op_pos_task_jacobian,op_pos_task_link_name,op_pos_task_pos_in_link);
	cout << op_pos_task_jacobian << endl;
	cout << robot->_M << endl; // Mass Matrix

	int wait;
	cin >> wait;

    return 0;
}

