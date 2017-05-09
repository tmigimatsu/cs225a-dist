#include "RobotCom.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <string>
#include <thread>
#include <map>

class PumaRedisDriver {

public:

	PumaRedisDriver() :
		command_data_(kDof),
		q_(kDof),
		dq_(kDof),
		Gamma_(kDof),
		kp_(kDof),
		kv_(kDof)
	{
		command_data_.setZero();
		q_.setZero();
		dq_.setZero();
		Gamma_.setZero();
		kp_ = kDefaultKp * Eigen::VectorXd::Ones(kDof);
		kv_ = kDefaultKv * Eigen::VectorXd::Ones(kDof);
		kp_str_ = RedisClient::encodeEigenMatrixString(kp_);
		kv_str_ = RedisClient::encodeEigenMatrixString(kv_);
	}

	const int kControlFreq = 1000;         // 1kHz control loop
	const int kInitializationPause = 1e6;  // 1ms pause before starting control loop
	static const int kDof = 6;
	const int kTaskSize = 7;
	const int kDefaultKp = 400;
	const int kDefaultKv = 40;
	const std::string kDefaultControlModeStr = "FLOAT";
	const ControlMode kDefaultControlMode = FLOAT;

	const HiredisServerInfo kRedisServerInfo = {
		"127.0.0.1",  // hostname
		6379,         // port
		{ 1, 500000 } // timeout = 1.5 seconds
	};

	// Redis keys for reading
	const std::string KEY_CONTROL_MODE = "cs225a::robot::puma::tasks::control_mode";
	const std::string KEY_COMMAND_DATA = "cs225a::robot::puma::tasks::command_data";
	const std::string KEY_KP           = "cs225a::robot::puma::tasks::kp";
	const std::string KEY_KV           = "cs225a::robot::puma::tasks::kv";

	// Redis keys for writing
	const std::string KEY_JOINT_POSITIONS  = "cs225a::robot::puma::sensors::q";
	const std::string KEY_JOINT_VELOCITIES = "cs225a::robot::puma::sensors::dq";
	const std::string KEY_JOINT_TORQUES    = "cs225a::robot::puma::actuators::fgc";

	// FLOAT
	// NJHOLD
	// JHOLD
	// NJMOVE
	// JMOVE
	// NJGOTO
	// JGOTO
	// NJTRACK
	// JTRACK
	// NXTRACK
	// XTRACK
	// NHOLD
	// HOLD
	// NGOTO
	// GOTO
	// NTRACK
	// TRACK
	// PFMOVE
	// LINE
	// PROJ1
	// PROJ2
	// PROJ3
	const std::map<std::string, ControlMode> kControlModeMap = {
		{"FLOAT", FLOAT},
		{"NJHOLD", NJHOLD},   {"JHOLD", JHOLD},
		{"NHOLD", NHOLD},     {"HOLD", HOLD},
		{"NJMOVE", NJMOVE},   {"JMOVE", JMOVE},
		{"NJGOTO", NJGOTO},   {"JGOTO", JGOTO},
		{"NJTRACK", NJTRACK}, {"JTRACK", JTRACK},
		{"NXTRACK", NXTRACK}, {"XTRACK", XTRACK},
		{"NGOTO", NGOTO},     {"GOTO", GOTO},
		{"NTRACK", NTRACK},   {"TRACK", TRACK},
		{"PFMOVE", PFMOVE},
		{"LINE", LINE},
		{"PROJ1", PROJ1}, {"PROJ2", PROJ2}, {"PROJ3", PROJ3}
	};

	void init();
	void run();

	volatile bool runloop_ = true;

	void eigenVectorFromBuffer(Eigen::VectorXd& vector);
	void eigenVectorToBuffer(const Eigen::VectorXd& vector);

	std::unique_ptr<RobotCom> puma_robot_;

protected:

	LoopTimer timer_;
	RedisClient redis_client_;

	ControlMode control_mode_ = kDefaultControlMode;
	Eigen::VectorXd command_data_;
	Eigen::VectorXd q_;
	Eigen::VectorXd dq_;
	Eigen::VectorXd Gamma_;
	Eigen::VectorXd kp_;
	Eigen::VectorXd kv_;
	std::string kp_str_;
	std::string kv_str_;

	float data_buffer_[kDof] = {0};

};
