#ifndef DEMO_PROJECT_H
#define DEMO_PROJECT_H

// CS225a includes
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

// External includes
#include <Eigen/Core>
#include <hiredis/hiredis.h>
#include <model/ModelInterface.h>

// std includes
#include <string>

class DemoProject {

	public:
		DemoProject(std::shared_ptr<Model::ModelInterface> robot,
		            const std::string &robot_name) :
			robot(robot),
			robot_name(robot_name),
			dof(robot->dof()),
			JOINT_TORQUES_COMMANDED_KEY(REDIS_KEY_PREFIX + robot_name + "::actuators::fgc"),
			JOINT_ANGLES_KEY           (REDIS_KEY_PREFIX + robot_name + "::sensors::q"),
			JOINT_VELOCITIES_KEY       (REDIS_KEY_PREFIX + robot_name + "::sensors::dq"),
			TIMESTAMP_KEY              (REDIS_KEY_PREFIX + robot_name + "::timestamp"),
			KP_POSITION_KEY            (REDIS_KEY_PREFIX + robot_name + "::tasks::kp_pos"),
			KV_POSITION_KEY            (REDIS_KEY_PREFIX + robot_name + "::tasks::kv_pos"),
			KP_ORIENTATION_KEY         (REDIS_KEY_PREFIX + robot_name + "::tasks::kp_ori"),
			KV_ORIENTATION_KEY         (REDIS_KEY_PREFIX + robot_name + "::tasks::kv_ori"),
			KP_JOINT_KEY               (REDIS_KEY_PREFIX + robot_name + "::tasks::kp_joint"),
			KV_JOINT_KEY               (REDIS_KEY_PREFIX + robot_name + "::tasks::kv_joint"),
			Jv(3, dof),
			N(dof, dof),
			Lambda_x(3, 3),
			command_torques(dof),
			g(dof),
			q_des(dof),
			q_err(dof),
			dq_des(dof),
			dq_err(dof),
			ddq(dof),
			controller_state(REDIS_SYNCHRONIZATION)
		{
			command_torques.setZero();
			q_des << 0, 30, 0, 60, 0, 90, 0;
			q_des *= M_PI / 180.0;
			dq_des.setZero();
			x_des << -0.1, 0.4, 0.7;
			dx_des.setZero();
		}

		void initialize();
		void runLoop();

	protected:
		enum state_t {
			REDIS_SYNCHRONIZATION,
			JOINT_SPACE_INITIALIZATION,
			OP_SPACE_POSITION_CONTROL
		};

		enum controller_status_t {
			RUNNING,
			FINISHED
		};

		void readRedisValues();
		void updateModel();
		void writeRedisValues();
		controller_status_t computeJointSpaceControlTorques();
		controller_status_t computeOperationalSpaceControlTorques();

		std::shared_ptr<Model::ModelInterface> robot;
		const std::string robot_name;
		const int dof;

		Eigen::MatrixXd Jv, N, Lambda_x;
		Eigen::VectorXd command_torques, F_posture, g, q_des, q_err, dq_des, dq_err, ddq;
		Eigen::Vector3d F_x, x, x_des, x_err, dx, dx_des, dx_err, ddx;

		double kp_pos = 40;
		double kv_pos = 10;
		double kp_ori = 40;
		double kv_ori = 10;
		double kp_joint = 40;
		double kv_joint = 10;

		const double TOLERANCE_Q_INIT = 0.1;
		const double TOLERANCE_DQ_INIT = 0.1;
		const double MAX_VELOCITY = 1.0;

		unsigned long long controller_counter = 0;
		double t_curr;
		state_t controller_state;

		const std::string REDIS_KEY_PREFIX = "cs225a::robot::";

		// Redis keys:
		// - write:
		const std::string JOINT_TORQUES_COMMANDED_KEY;

		// - read:
		const std::string JOINT_ANGLES_KEY;
		const std::string JOINT_VELOCITIES_KEY;
		const std::string TIMESTAMP_KEY;
		const std::string KP_POSITION_KEY;
		const std::string KV_POSITION_KEY;
		const std::string KP_ORIENTATION_KEY;
		const std::string KV_ORIENTATION_KEY;
		const std::string KP_JOINT_KEY;
		const std::string KV_JOINT_KEY;
		const std::string KP_JOINT_INIT_KEY;
		const std::string KV_JOINT_INIT_KEY;

		const int CONTROL_FREQ = 1000;
		const int INITIALIZATION_PAUSE = 1e6;
		LoopTimer timer;

		const HiredisServerInfo redisServerInfo = {
			"127.0.0.1",  // hostname
			6379,         // port
			{ 1, 500000 } // timeout = 1.5 seconds
		};
		RedisClient redis_client;
		std::string redis_buf;

};

#endif //DEMO_PROJECT_H
