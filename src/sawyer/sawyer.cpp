#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <fstream>
#include <string>

#include <signal.h>
static volatile bool runloop = true;
void stop(int) { runloop = false; }

using namespace std;

// Location of URDF files specifying world and robot information
static string world_file = "";
static string robot_file = "";
static string robot_name = "";

static unsigned long long controller_counter = 0;

// Redis keys:
// - write:
static std::string JOINT_TORQUES_COMMANDED_KEY = "";

// - read:
static std::string JOINT_ANGLES_KEY  = "";
static std::string JOINT_VELOCITIES_KEY = "";
static std::string TIMESTAMP_KEY = "";
static std::string KP_POSITION_KEY = "";
static std::string KV_POSITION_KEY = "";
static std::string KP_ORIENTATION_KEY = "";
static std::string KV_ORIENTATION_KEY = "";
static std::string KP_JOINT_KEY = "";
static std::string KV_JOINT_KEY = "";
static std::string KP_JOINT_INIT_KEY = "";
static std::string KV_JOINT_INIT_KEY = "";
static std::string EE_POSITION_KEY = "";
static std::string EE_DES_POSITION_KEY = "";

// Function to parse command line arguments
void parseCommandline(int argc, char** argv);

int main(int argc, char** argv) {

	// Parse command line and set redis keys
	parseCommandline(argc, argv);
	JOINT_TORQUES_COMMANDED_KEY = "cs225a::robot::" + robot_name + "::actuators::fgc";
	JOINT_ANGLES_KEY            = "cs225a::robot::" + robot_name + "::sensors::q";
	JOINT_VELOCITIES_KEY        = "cs225a::robot::" + robot_name + "::sensors::dq";
	TIMESTAMP_KEY               = "cs225a::robot::" + robot_name + "::timestamp";
	KP_POSITION_KEY             = "cs225a::robot::" + robot_name + "::tasks::kp_pos";
	KV_POSITION_KEY             = "cs225a::robot::" + robot_name + "::tasks::kv_pos";
	KP_ORIENTATION_KEY          = "cs225a::robot::" + robot_name + "::tasks::kp_ori";
	KV_ORIENTATION_KEY          = "cs225a::robot::" + robot_name + "::tasks::kv_ori";
	KP_JOINT_KEY                = "cs225a::robot::" + robot_name + "::tasks::kp_joint";
	KV_JOINT_KEY                = "cs225a::robot::" + robot_name + "::tasks::kv_joint";
	KP_JOINT_INIT_KEY           = "cs225a::robot::" + robot_name + "::tasks::kp_joint_init";
	KV_JOINT_INIT_KEY           = "cs225a::robot::" + robot_name + "::tasks::kv_joint_init";
	EE_POSITION_KEY             = "cs225a::robot::" + robot_name + "::tasks::ee_pos";
	EE_DES_POSITION_KEY	    = "cs225a::robot::" + robot_name + "::tasks::ee_pos_des";

	cout << "Loading URDF world model file: " << world_file << endl;

	// Start redis client
	// Make sure redis-server is running at localhost with default port 6379
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// Load robot
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);
	robot->updateModel();
	const int dof = robot->dof();
	// Create a loop timer
	const double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1e6); // 1 ms pause before starting loop
	
	Eigen::VectorXd command_torques(dof);
	Eigen::VectorXd q_err(dof), g(dof);
	Eigen::VectorXd q_initial = Eigen::VectorXd::Zero(dof); // Desired initial joint position

	Eigen::MatrixXd J, Lambda(6,6), N(dof,dof);
	Eigen::Matrix3d R, R_des;
	Eigen::Vector3d x, x_des, dx, p, w, d_phi, ddx, dw;
	Eigen::VectorXd nullspace_damping, q_des(dof), ddx_dw(6), F(6);
	Eigen::Vector3d x_initial;
	x_des << 0.5, 0, 0.8;

	redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);

	//q_des = robot->_q;
	q_des = q_initial;	

	int kp_pos = 100;
	int kv_pos = 35;
	int kp_ori = 100;
	int kv_ori = 50;
	int kv_joint = 20;
	int kp_joint = 30;
	double kMaxVelocity = 0.3;
	string redis_buf;
	double t_curr = 0;
	robot->position(x_initial, "right_l6", Eigen::Vector3d::Zero());
	robot->rotation(R_des, "right_l6");
	while (runloop) {
		timer.waitForNextLoop();

		// Get current simulation timestamp from Redis
		redis_client.getCommandIs(TIMESTAMP_KEY, redis_buf);
		if (redis_buf.length() != 0)
		{
			t_curr = stod(redis_buf);
		}

		// read from Redis current sensor values
		redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

		// Update the model
		robot->updateModel();
		robot->gravityVector(g);
		//------ Compute controller torques
		robot->J(J, "right_l6", Eigen::Vector3d::Zero());
		robot->taskInertiaMatrixWithPseudoInv(Lambda, J);
		robot->gravityVector(g);
		robot->position(x, "right_l6", Eigen::Vector3d::Zero());
		robot->rotation(R, "right_l6");
		robot->linearVelocity(dx, "right_l6", Eigen::Vector3d::Zero());
		robot->angularVelocity(w, "right_l6");
		robot->orientationError(d_phi, R_des, R);
		robot->nullspaceMatrix(N, J);

		x_des << -0.2, 0.2 * sin(0.1*M_PI*t_curr) - 0.1, 0.2 + 0.2 * cos(0.1*M_PI*t_curr);
		x_des += x_initial;

		// Send end effector position for trajectory visualization
		redis_client.setEigenMatrixDerivedString(EE_POSITION_KEY, x);
		redis_client.setEigenMatrixDerivedString(EE_DES_POSITION_KEY, x_des);
		
		// Orientation and position controller with pose and damping in the nullspace
		dw = -kp_ori * d_phi - kv_ori * w;

		// Velocity Control
		Eigen::Vector3d x_err = x - x_des;
		Eigen::Vector3d dx_des = -(kp_pos / kv_pos) * x_err;
		double v = kMaxVelocity / dx_des.norm();
		if (v > 1) v = 1;
		Eigen::Vector3d dx_err = dx - v * dx_des;
		ddx = -kv_pos * dx_err;

		ddx_dw << dw, ddx;
		F = Lambda * ddx_dw;
		nullspace_damping = N.transpose() * robot->_M * (kp_joint * (q_des - robot->_q) - kv_joint * robot->_dq);
		command_torques = J.transpose() * F + nullspace_damping + g; // Don't forget to add or remove gravity for real robot
		// command_torques = robot->_M * (kp_joint * (q_des - robot->_q) - kv_joint * robot->_dq); // Joint Control Law

		redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	}
	
	command_torques.setZero();
	redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	return 0;
}


//------------------------------------------------------------------------------
void parseCommandline(int argc, char** argv) {
	if (argc != 4) {
		cout << "Usage: hw2 <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
		exit(0);
	}
	// argument 0: executable name
	// argument 1: <path-to-world.urdf>
	world_file = string(argv[1]);
	// argument 2: <path-to-robot.urdf>
	robot_file = string(argv[2]);
	// argument 3: <robot-name>
	robot_name = string(argv[3]);
}
