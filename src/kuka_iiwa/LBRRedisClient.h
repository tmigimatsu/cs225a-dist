/**
* \copyright
* TaCo - Task-Space Control Library.<br>
* Copyright (c) 2015-2016
*<br>
* TaCo is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*<br>
* TaCo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*<br>
* You should have received a copy of the Lesser GNU Lesser General Public License
* along with TaCo.  If not, see <http://www.gnu.org/licenses/>.
<br>
Author: Brian Soe <bsoe@stanford.edu>
Author: Mikael Jorda <mjorda@stanford.edu>
*/


#ifndef SAI_KUKA_IIWA_DRIVER_LBRREDISCLIENT_H_
#define SAI_KUKA_IIWA_DRIVER_LBRREDISCLIENT_H_

#include "friLBRClient.h"
#include "redis/RedisClient.h"
#include <ButterworthFilter.h>
#include <fstream>
#include <string>

#ifdef USE_KUKA_LBR_DYNAMICS
    #include <KukaLBRDynamics/Robot.h>
    #include <Eigen/Dense>
    #include <rbdl/Model.h>
    #ifndef RBDL_BUILD_ADDON_URDFREADER
      #error "Error: RBDL addon urdfmodel not enabled."
    #endif
    #include <rbdl/addons/urdfreader/urdfreader.h>
#endif

namespace KukaIIWA {
	// Redis namespace
	const std::string kRedisKeyPrefix = "cs225a::kuka_iiwa::";

	// Read keys
	const std::string KEY_COMMAND_TORQUES  = kRedisKeyPrefix + "actuators::fgc";

	// Write keys
	const std::string KEY_SENSOR_TORQUES   = kRedisKeyPrefix + "sensors::torques";
	const std::string KEY_JOINT_POSITIONS  = kRedisKeyPrefix + "sensors::q";
	const std::string KEY_JOINT_VELOCITIES = kRedisKeyPrefix + "sensors::dq";

	const std::string REDIS_IP = RedisServer::DEFAULT_IP;
	const int REDIS_PORT = RedisServer::DEFAULT_PORT;
}

namespace KUKA {
namespace FRI {


/** \brief Client for Kuka LBR IIWA that reads and writes to shared memory. */
class LBRRedisClient : public KUKA::FRI::LBRClient
{
   
public:
      
	LBRRedisClient(const std::string& redis_ip=RedisServer::DEFAULT_IP,
	               const int redis_port=RedisServer::DEFAULT_PORT);
   
   ~LBRRedisClient();
   
   /** \brief Callback for FRI state changes.
    * @param oldState
    * @param newState
    */
   virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);
    
   /** \brief Callback for the FRI session state 'Commanding Wait'. */
   virtual void waitForCommand();
   
   /** \brief Callback for the FRI state 'Commanding Active'. */
   virtual void command();

   /** \brief the joint names, this should match the robot model urdf and sdf */
   const std::vector<std::string> joint_names_ = {"j0","j1","j2","j3","j4","j5","j6"};

   /** \brief the joint maximum torques */
   const double torque_limit_[7] = {176, 176, 110, 110, 110, 40, 40}; // LBR iiwa 7 R800
   //const double max_torque_[7] = {320, 320, 176,176, 110, 40, 40}; // LBR iiwa 14 R820

   /** \brief the joint limits. LBR iiwa 7 R800 or LBR iiwa 1r R820 */
   const double joint_limit_[7] = {2.9670, 2.0944, 2.9670, 2.0944, 2.9670, 2.0944, 3.0543};

   /** \brief the jerk limit (max increment of torque per joint). TODO : find real jerk limits */
   const double jerk_limit_[7] = {8.8, 8.8, 5.5, 5.5, 5.5, 2, 2};

   /** \brief Velocity limits. TODO : find real velocity limits */
   // const double velocity_limit_[7] = {98.0/180.0*M_PI, 98.0/180.0*M_PI, 100.0/180.0*M_PI, 130.0/180.0*M_PI, 140.0/180.0*M_PI, 180.0/180.0*M_PI, 180.0/180.0*M_PI}/180*M_PI; // for LBR IIWA 7kg from specs
   const double velocity_limit_[7] = {90.0/180.0*M_PI, 90.0/180.0*M_PI, 95.0/180.0*M_PI, 125.0/180.0*M_PI, 135.0/180.0*M_PI, 170.0/180.0*M_PI, 170.0/180.0*M_PI};

   /** \brief boolean to go to damped exit */
   bool exit_program_ = false;

   /** \brief kv for damped exit */
   const double exit_kv_[7] = {8, 8, 5, 5, 5, 2, 2};

   /** \brief number of servo ticks for exiting for exiting */
   int exit_counter_ = 40;

   /** \brief Wrist position to check it does not go too close to the ground or the base of the robot */
   Eigen::Vector3d wrist_position_;

   /** \brief radius of safety around the base of the robot, in meters */
   // const double safe_radius_ = 0.5;

   /** \brief lowest allowed position of the wrist */
   const double lowest_wrist_pos_ = 0.45;

  /** \brief highest allowed position of the wrist */
   const double highest_wrist_pos_ = 1.05;

   /** \brief the cutoff frequency of the filter, in the range of (0 0.5) of sampling freq */
   const double cutoff_freq_ = 0.1;


protected:

   /** \brief print out command mode */
   void printCommandMode(KUKA::FRI::EClientCommandMode command_mode);

   /** \brief print robot state */
   void printRobotState();

   /** \brief Prints a message */
   void printMessage(const std::string& message);

   /** \brief returns the value of elapsed time as a double */
   double elapsedTime(timespec start, timespec end);

   /** \brief Redis Client and Info */
   CDatabaseRedisClient redis_client_;
   HiredisServerInfo info_;

   /** \brief time of previous measurement, for calculating dt and velocity */
   timespec prev_time_ = {0, 0};

   /** \brief last fri command mode, for checking for mode change */
   KUKA::FRI::EClientCommandMode prev_fri_command_mode_ = KUKA::FRI::NO_COMMAND_MODE;

   /** \brief velocity filter */
   sai::ButterworthFilter filter;

   /** \brief raw velocity */
   Eigen::VectorXd vel_raw_;

   /** \brief filtered velocity */
   Eigen::VectorXd vel_filtered_;

   /** \brief desired torque */
   double desired_torque_[LBRState::NUMBER_OF_JOINTS] = {0};

   /** \brief previous commanded torque */
   double prev_torque_[LBRState::NUMBER_OF_JOINTS] = {0};

   /** \brief desired position */
   double desired_position_[LBRState::NUMBER_OF_JOINTS] = {0};

   /** \brief position */
   double position_[KUKA::FRI::LBRState::NUMBER_OF_JOINTS] = {0};
   Eigen::Map<VectorXd> q_;

   /** \brief previous position */
   double prev_position_[KUKA::FRI::LBRState::NUMBER_OF_JOINTS] = {0};

   /** to compute the sine wave for the dither and friction compensation */
   double sampleTime_ = 0.0;
   double currentTime_ = 0.0;

   /** \brief dither in position to activate friction compensation */
   double dither_[LBRState::NUMBER_OF_JOINTS] = {0};

   /** \brief sensed torques */
   double sensed_torques_[LBRState::NUMBER_OF_JOINTS] = {0};

   unsigned long long counter_ = 0;


#ifdef USE_KUKA_LBR_DYNAMICS

   void parseTool();

   /** \brief name of the tool */
   const std::string tool_file_name_ = "../tool.xml";

   /** \brief tool mass */
   double tool_mass_ = 0;

   /** \brief tool mass */
   Eigen::Vector3d tool_com_ = Eigen::Vector3d::Zero();

   /** \brief kuka dynamics */
   kuka::Robot dynamics_;

   /** \brief rbdl dynamics for wrist safety implementation */
   RigidBodyDynamics::Model rbdl_model_;

   /** \brief path to the urdf model for rbdl dynamics */
   const std::string path_to_model_file_ = "resources/kuka_iiwa.urdf";

#endif

};

} //namespace FRI
} //namespace KUKA

#endif // SAI_KUKA_IIWA_DRIVER_LBRREDISCLIENT_H_
