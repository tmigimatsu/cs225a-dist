#ifndef SAI2_KINOVA_REDIS_DRIVER_H
#define SAI2_KINOVA_REDIS_DRIVER_H

#include "KinovaJaco.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include "KinovaTypes.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"

#include <string>
#include <Eigen/Core>

static const float FINGERS_OPEN = 0; // value of fingers when completely open
static const float FINGERS_CLOSE = 6250; // value of fingers when completely closed

class KinovaJacoRedisDriver {

public:

	KinovaJacoRedisDriver(const std::string& redis_ip, const int redis_port,
	                      const std::string& gravity_filename);
	void connect();
	void run();

protected:

	/***** Constants *****/
	float kGravity[OPTIMAL_Z_PARAM_SIZE];
	const int kControlFreq = 1000;         // 1 kHz control loop
	const int kInitializationPause = 1e6;  // 1ms pause before starting control loop

	/***** Member Variables *****/
	RedisClient redis_;
	LoopTimer timer_;

	// Control mode
	KinovaJaco::ControlMode control_mode_ = KinovaJaco::READ_POSITIONS;

	// Command joint positions
	TrajectoryPoint command_joint_positions_;

	// Command torques
	float arr_command_torques_[KinovaJaco::DOF] = {0};
	Eigen::Map<Eigen::VectorXf> command_torques_ = Eigen::Map<Eigen::VectorXf>(arr_command_torques_, KinovaJaco::DOF);

	/***** Private Member Functions *****/
	void parseGravity(const std::string& gravity_filename);
	void readRedisValues();
	void writeRedisValues();
	void sendRobotCommand();

};

#endif  // SAI2_KINOVA_REDIS_DRIVER_H
