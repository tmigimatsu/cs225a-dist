// This example application runs a controller for the IIWA

#include "Model/ModelInterface.h"
#include "redis/CDatabaseRedisClient.hpp"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void stop(int){runloop = false;}

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/RRPbot.urdf";
const string robot_name = "RRPbot";

unsigned long long controller_counter = 0;
std::string fgc_command_enabled = "";

// redis keys:
// - write:
// const std::string JOINT_ANGLES_DES_KEY  = "scl::robot::iiwaBot::sensors::q_des";
const std::string JOINT_TORQUES_COMMANDED_KEY = "scl::robot::iiwaBot::actuators::fgc";
// - read:
const std::string JOINT_ANGLES_KEY  = "scl::robot::iiwaBot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "scl::robot::iiwaBot::sensors::dq";
const std::string FGC_ENABLE_KEY  = "scl::robot::iiwaBot::fgc_command_enabled";
// const std::string JOINT_TORQUES_KEY = "scl::robot::iiwaBot::sensors::fgc";
// - debug
const std::string POS_TASK_X_DESIRED_KEY = "scl::robot::iiwaBot::pos_task_x_desired";
const std::string POS_TASK_X_KEY = "scl::robot::iiwaBot::pos_task_x";
const std::string POS_TASK_FORCE_KEY = "scl::robot::iiwaBot::pos_task_force";
const std::string POS_TASK_JOINT_TORQUES_KEY = "scl::robot::iiwaBot::pos_task_joint_torques";
const std::string ORI_TASK_R_DESIRED_KEY = "scl::robot::iiwaBot::ori_task_R_desired";
const std::string ORI_TASK_R_KEY = "scl::robot::iiwaBot::ori_task_R";
const std::string ORI_TASK_FORCE_KEY = "scl::robot::iiwaBot::ori_task_force";
const std::string ORI_TASK_JOINT_TORQUES_KEY = "scl::robot::iiwaBot::ori_task_joint_torques";
const std::string JOINT_TASK_Q_DESIRED_KEY = "scl::robot::iiwaBot::joint_task_q_desired";
const std::string JOINT_TASK_TORQUES_KEY = "scl::robot::iiwaBot::gui::joint_task_torques";
// GUI
const std::string POS_KP_KEY = "scl::robot::iiwaBot::gui::pos_kp";
const std::string POS_KV_KEY = "scl::robot::iiwaBot::gui::pos_kv";
const std::string ORI_KP_KEY = "scl::robot::iiwaBot::gui::ori_kp";
const std::string ORI_KV_KEY = "scl::robot::iiwaBot::gui::ori_kv";
const std::string JOINT_KP_KEY = "scl::robot::iiwaBot::gui::joint_kp";
const std::string JOINT_KV_KEY = "scl::robot::iiwaBot::gui::joint_kv";

 void sighandler(int sig)
 { runloop = false; }

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = CDatabaseRedisClient();
	redis_client.serverIs(info);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	// read from Redis
	redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	robot->updateModel();

	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);

	Eigen::MatrixXd N_prec, N_tmp;

	// Joint control
	Eigen::VectorXd joint_task_desired_position(dof), joint_task_torques(dof);

	joint_task_desired_position << 90.0/180.0*M_PI, 30.0/180.0*M_PI, 0.0, 60.0/180.0*M_PI, 0.0, 90.0/180.0*M_PI, 0.0;
	redis_client.setEigenMatrixDerivedString(JOINT_TASK_Q_DESIRED_KEY,joint_task_desired_position);

	double joint_kp = 100.0;
	double joint_kv = 20.0;
	redis_client.setCommandIs(JOINT_KP_KEY,std::to_string(joint_kp));
	redis_client.setCommandIs(JOINT_KV_KEY,std::to_string(joint_kv));

	// operational space position task
	std::string op_pos_task_link_name = "link3";
	Eigen::Vector3d op_pos_task_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.0);
	Eigen::Vector3d op_pos_task_x, op_pos_task_dx, op_pos_task_x_desired, op_pos_task_f;
	Eigen::MatrixXd op_pos_task_jacobian(3,dof), op_pos_task_J(3,dof);
	Eigen::MatrixXd Lambda_pos(3,3);
	// Eigen::MatrixXd op_pos_task_N(dof,dof);
	Eigen::VectorXd op_pos_task_torques(dof);

	Eigen::Matrix3d op_pos_kp = 100.0*Eigen::Matrix3d::Identity();
	Eigen::Matrix3d op_pos_kv = 20.0*Eigen::Matrix3d::Identity();
	redis_client.setEigenMatrixDerivedString(POS_KP_KEY, op_pos_kp);
	redis_client.setEigenMatrixDerivedString(POS_KV_KEY, op_pos_kv);

	// robot->Jv(op_pos_task_jacobian,op_pos_task_link_name,op_pos_task_pos_in_link);
	Eigen::Vector3d initial_position;
	robot->position(initial_position,op_pos_task_link_name,op_pos_task_pos_in_link);
	// op_pos_task_x_desired = initial_position + Eigen::Vector3d(0.0,0.0,-0.15);
	redis_client.setEigenMatrixDerivedString(POS_TASK_X_DESIRED_KEY,initial_position);

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	// enable the controller
	fgc_command_enabled = "1";
	redis_client.setCommandIs(FGC_ENABLE_KEY,fgc_command_enabled);

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

		// update the model 20 times slower
		if(controller_counter%20 == 0)
		{
			robot->updateModel();

			/////////////////////////// update jacobians, mass matrices and nullspaces
			N_prec = Eigen::MatrixXd::Identity(dof,dof);
			N_tmp = Eigen::MatrixXd::Identity(dof,dof);

			// position controller 
			robot->Jv(op_pos_task_jacobian,op_pos_task_link_name,op_pos_task_pos_in_link);
			op_pos_task_J = op_pos_task_jacobian*N_prec;
			robot->taskInertiaMatrix(Lambda_pos,op_pos_task_J);
			robot->nullspaceMatrix(N_prec,op_pos_task_J,N_tmp);
			N_tmp = N_prec;

			// orientation controller
			robot->Jw(op_ori_task_jacobian,op_ori_task_link_name);
			op_ori_task_J = op_ori_task_jacobian*N_prec;
			robot->taskInertiaMatrix(Lambda_ori,op_ori_task_J);
			robot->nullspaceMatrix(N_prec,op_ori_task_J,N_tmp);
		}

		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;
		//---- position controller 
		// find current position
		robot->position(op_pos_task_x,op_pos_task_link_name,op_pos_task_pos_in_link);
		op_pos_task_dx = op_pos_task_J*robot->_dq;
		redis_client.setEigenMatrixDerivedString(POS_TASK_X_KEY, op_pos_task_x);
		// update desired position
		redis_client.getEigenMatrixDerivedString(POS_TASK_X_DESIRED_KEY,op_pos_task_x_desired); 
		// compute joint torques
		redis_client.getEigenMatrixDerivedString(POS_KP_KEY, op_pos_kp);
		redis_client.getEigenMatrixDerivedString(POS_KV_KEY, op_pos_kv);

		op_pos_task_f = Lambda_pos*(-op_pos_kp*(op_pos_task_x - op_pos_task_x_desired) - op_pos_kv*op_pos_task_dx);
		op_pos_task_torques = op_pos_task_J.transpose()*op_pos_task_f;

		redis_client.setEigenMatrixDerivedString(POS_TASK_FORCE_KEY, op_pos_task_f);
		redis_client.setEigenMatrixDerivedString(POS_TASK_JOINT_TORQUES_KEY, op_pos_task_torques);

		//---- orientation controller
		// find current orientation
		robot->rotation(op_ori_task_R,op_ori_task_link_name);
		op_ori_task_w = op_ori_task_J*robot->_dq;
		redis_client.setEigenMatrixDerivedString(ORI_TASK_R_KEY, op_ori_task_R);
		// update desired orientation
		redis_client.getEigenMatrixDerivedString(ORI_TASK_R_DESIRED_KEY,op_ori_task_R_desired);
		// find orientation error
		Eigen::Vector3d d_phi;
		robot->orientationError(d_phi,op_ori_task_R_desired,op_ori_task_R);
		// compute joint torques
		redis_client.getEigenMatrixDerivedString(ORI_KP_KEY, op_ori_kp);
		redis_client.getEigenMatrixDerivedString(ORI_KV_KEY, op_ori_kv);
		op_ori_task_f = Lambda_ori*(-op_ori_kp*d_phi - op_ori_kv*op_ori_task_w);
		op_ori_task_torques = op_ori_task_J.transpose()*op_ori_task_f;

		redis_client.setEigenMatrixDerivedString(ORI_TASK_FORCE_KEY, op_ori_task_f);
		redis_client.setEigenMatrixDerivedString(ORI_TASK_JOINT_TORQUES_KEY, op_ori_task_torques);

		//----- Joint controller 
		redis_client.getEigenMatrixDerivedString(JOINT_TASK_Q_DESIRED_KEY, joint_task_desired_position);
		Eigen::VectorXd tmp = Eigen::VectorXd(1);
		redis_client.getEigenMatrixDerivedString(JOINT_KP_KEY, tmp);
		joint_kp = tmp(0);
		redis_client.getEigenMatrixDerivedString(JOINT_KV_KEY, tmp);
		joint_kv = tmp(0);
		joint_task_torques = robot->_M*(-joint_kp*(robot->_q - joint_task_desired_position) - joint_kv*robot->_dq);

		redis_client.setEigenMatrixDerivedString(JOINT_TASK_TORQUES_KEY, N_prec.transpose()*joint_task_torques);

		//------ Final torques
		command_torques = op_pos_task_torques + op_ori_task_torques + N_prec.transpose()*joint_task_torques;


		redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

		/////////// TODO : fix this
		// redis_client.getCommandIs(FGC_ENABLE_KEY, fgc_command_enabled);
		// std::cout << fgc_command_enabled << '\n';
		if(fgc_command_enabled == "0")
		{
			runloop = false;
		}

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    fgc_command_enabled = "0";
    redis_client.setCommandIs(FGC_ENABLE_KEY,fgc_command_enabled);

    return 0;
}

