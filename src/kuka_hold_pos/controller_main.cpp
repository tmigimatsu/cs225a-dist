// This example application runs a controller for the IIWA

#include "model/ModelInterface.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void stop(int){runloop = false;}

using namespace std;

const string world_file = "resources/kuka_hold_pos/world.urdf";
const string robot_file = "resources/kuka_hold_pos/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";

unsigned long long controller_counter = 0;

// redis keys:
// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::KUKA_IIWA::actuators::fgc";
// - read:
const std::string JOINT_ANGLES_KEY  = "sai2::KUKA_IIWA::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::KUKA_IIWA::sensors::dq";
const std::string JOINT_TORQUES_SENSED_KEY = "sai2::KUKA_IIWA::sensors::torques";

 void sighandler(int sig)
 { runloop = false; }

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, true);

	// read from Redis
	redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	robot->updateModel();

	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);

	Eigen::MatrixXd N_prec;

	// Joint control
	Eigen::VectorXd joint_task_desired_position(dof), joint_task_torques(dof);
	Eigen::VectorXd initial_joint_position(dof);
	initial_joint_position = robot->_q;

	Eigen::MatrixXd joint_kp = Eigen::MatrixXd::Zero(7,7);
	Eigen::MatrixXd joint_kv = Eigen::MatrixXd::Zero(7,7);
	Eigen::VectorXd joint_kp_vector = Eigen::VectorXd::Zero(7);
	Eigen::VectorXd joint_kv_vector = Eigen::VectorXd::Zero(7);
	joint_kp_vector << 100, 100, 60, 60, 10, 8, 5;
	joint_kv_vector << 20, 20, 10, 10, 5, 3, 1;

	joint_kp = joint_kp_vector.asDiagonal();
	joint_kv = joint_kv_vector.asDiagonal();

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

		// update the model 20 times slower
		if(controller_counter%20 == 0)
		{
			robot->updateModel();

		}

		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		//----- Joint control
		joint_task_torques = ( -joint_kp*(robot->_q - initial_joint_position) - joint_kv*robot->_dq);

		//------ Final torques
		command_torques = joint_task_torques;

		//------ senf torques to robot
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}
