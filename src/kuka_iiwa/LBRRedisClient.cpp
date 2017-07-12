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


#include "LBRRedisClient.h"
#include <sstream>
#include <tinyxml2.h>

namespace KUKA {
namespace FRI {

LBRRedisClient::LBRRedisClient(const std::string& redis_ip, const int redis_port)
	: q_(position_, LBRState::NUMBER_OF_JOINTS),
	  Gamma_command_(desired_torque_, LBRSTATE::NUMBER_OF_JOINTS),
	  Gamma_sensor_(sensed_torques_< LBRState::NUMBER_OF_JOINTS)
#ifdef USE_KUKA_LBR_DYNAMICS
	, dynamics_(kuka::Robot::LBRiiwa)
#endif
{
	filter.setDimension(LBRState::NUMBER_OF_JOINTS);
	filter.setCutoffFrequency(cutoff_freq_);

	vel_raw_.setZero(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
	vel_filtered_.setZero(KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

	wrist_position_.setZero();

    redis_ = RedisClient();
	redis_.connect(redis_ip, redis_port);

#ifdef USE_KUKA_LBR_DYNAMICS
	parseTool();

	// parse rbdl model from urdf
    bool success = RigidBodyDynamics::Addons::URDFReadFromFile(path_to_model_file_.c_str(), &rbdl_model_, false, false);
    if (!success) 
    {
		std::cout << "Error loading model [" + path_to_model_file_ + "]" << "\n";
		exit(0);
    }
#endif
}


LBRRedisClient::~LBRRedisClient()
{  
	// delete kuka_model_; 
}


void LBRRedisClient::onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState)
{
	LBRClient::onStateChange(oldState, newState);
	// react on state change events
	switch (newState)
	{
		case KUKA::FRI::IDLE:
		{
			printMessage("FRI switched to state [IDLE]\n");
			break;
		} 
		case KUKA::FRI::MONITORING_WAIT:
		{
			printMessage("FRI switched to state [MONITORING_WAIT]\n");
			break;
		}       
		case KUKA::FRI::MONITORING_READY:
		{
			sampleTime_ = robotState().getSampleTime();
			printMessage("FRI switched to state [MONITORING_READY]\n");
			break;
		}
		case KUKA::FRI::COMMANDING_WAIT:
		{
			printMessage("FRI switched to state [COMMANDING_WAIT]\n");
			break;
		}   
		case KUKA::FRI::COMMANDING_ACTIVE:
		{
			printMessage("FRI switched to state [COMMANDING_ACTIVE]\n");
			break;
		}   
		default:
		{
			printMessage("FRI switched to state [UNKNOWN]\n");
			break;
		}
	}
}


void LBRRedisClient::waitForCommand()
{
	// In waitForCommand(), the joint values have to be mirrored. Which is done, 
	// by calling the base method.
	LBRClient::waitForCommand();

	// If we want to command torques, we have to command them all the time; even in
	// waitForCommand(). This has to be done due to consistency checks. In this state it is
	// only necessary, that some torque values are sent. The LBR does not take the
	// specific value into account.
	if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE) 
	{
		robotCommand().setJointPosition(robotState().getIpoJointPosition());
		robotCommand().setTorque(desired_torque_);
	}
}


void LBRRedisClient::command()
{
	// In command(), the joint values have to be sent. Which is done by calling
	// the base method.
	LBRClient::command();

	// check for control mode change
	KUKA::FRI::EClientCommandMode fri_command_mode = robotState().getClientCommandMode();
	if (fri_command_mode != prev_fri_command_mode_)
	{
		printCommandMode(fri_command_mode);
		prev_fri_command_mode_ = fri_command_mode;
	}

	// get the position and measure torque
	memcpy(position_, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
	memcpy(sensed_torques_, robotState().getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

	// get the time
	timespec time;
	time.tv_sec = robotState().getTimestampSec();
	time.tv_nsec = robotState().getTimestampNanoSec();

	// get the velocity
	if (prev_time_.tv_sec==0 && prev_time_.tv_nsec==0)
	{
		// if not initialized, set to zero
		vel_raw_.setZero();
		vel_filtered_.setZero();
	} 
	else 
	{
		// velocity = (position - last position)/(time - last time)
		double dt = elapsedTime(prev_time_, time);
		for (int i=0; i<LBRState::NUMBER_OF_JOINTS; ++i)
		{
			vel_raw_(i) = (position_[i] - prev_position_[i])/dt;
		}
	vel_filtered_ = filter.update(vel_raw_);
	}

	// Send positions, velocities and sensed torques
	redis_client_.setDoubleArray(JOINT_ANGLES_KEY, position_, 7);
	redis_client_.setEigenMatrixDerived(JOINT_VELOCITIES_KEY, vel_filtered_);
	redis_client_.setDoubleArray(JOINT_TORQUES_SENSED_KEY, sensed_torques_, 7);

	// get commanded torques
	for(int i=0; i<7; i++) // First, zero out the array
	{
		desired_torque_[i] = 0;
	}
	redis_client_.getDoubleArray(JOINT_TORQUES_COMMANDED_KEY, desired_torque_, 7);	

	// if(counter_%100 == 0)
	// {
	// std::cout << "desired torque read from redis : ";
	// for(int i=0; i<7; i++)
	// {
	// 	std::cout << " " << desired_torque_[i];
	// }
	// std::cout << std::endl;
	// }

	if(exit_program_)
	{
		if(exit_counter_ <= 0)
		{
			exit(0);
		}
		for(int i=0; i<7; i++)
		{
			desired_torque_[i] = -exit_kv_[i]*vel_filtered_[i];
		}
		exit_counter_--;
	}


	// ADD GRAVITY COMPENSATION FOR TOOL
	#ifdef USE_KUKA_LBR_DYNAMICS
	if (fri_command_mode==KUKA::FRI::TORQUE)
	{
		Eigen::VectorXd q(LBRState::NUMBER_OF_JOINTS);
		Eigen::VectorXd dq(LBRState::NUMBER_OF_JOINTS);
		for (int i=0; i<LBRState::NUMBER_OF_JOINTS; ++i)
		{
			q(i) = position_[i];
			dq(i) = vel_raw_[i];
		}
		Eigen::MatrixXd J(6, LBRState::NUMBER_OF_JOINTS); J.setZero();
		dynamics_.getJacobian(J, q, tool_com_, 7);
		Eigen::VectorXd tool_g_force(6); tool_g_force << 0,0,-9.81*tool_mass_,0,0,0;
		Eigen::VectorXd tool_g_torque = J.transpose()*tool_g_force;
		for (int i=0; i<LBRState::NUMBER_OF_JOINTS; ++i)
		{
			desired_torque_[i] -= tool_g_torque(i);
		}
	}
	#endif

	// COMPENSATE FOR THE OFFSET IN THE FIRST JOINT
	if (fri_command_mode==KUKA::FRI::TORQUE)
	{
		desired_torque_[0] -= 0.3;
	}

	// CHECK COMMAND
	switch(fri_command_mode)
	{
		case KUKA::FRI::NO_COMMAND_MODE:
		break;
		case KUKA::FRI::POSITION:
		for (int i=0; i<LBRState::NUMBER_OF_JOINTS; ++i)
		{
			if (desired_position_[i] < -joint_limit_[i])
			{
				desired_position_[i] = -joint_limit_[i];
				printMessage("WARNING : desired position of joint ["+std::to_string(i)+"] exceeds limit\n");
			}
			else if (desired_position_[i] > joint_limit_[i])
			{
				desired_position_[i] = joint_limit_[i];
				printMessage("WARNING : desired position of joint ["+std::to_string(i)+"] exceeds limit\n");
			}
		}
		break;
		case KUKA::FRI::WRENCH:
		printMessage("Wrench control mode not supported by the driver");
		break;
		case KUKA::FRI::TORQUE:

		// ********************************************************************
		// Torque mode safeties here
		// ********************************************************************

		// Torque saturation
		for (int i=0; i<LBRState::NUMBER_OF_JOINTS; ++i)
		{
			if (desired_torque_[i] < -torque_limit_[i])
			{
				desired_torque_[i] = -torque_limit_[i];
				printMessage("WARNING : desired torque of joint ["+std::to_string(i)+"] exceeds limit\n");
			}
			else if (desired_torque_[i] > torque_limit_[i])
			{
				desired_torque_[i] = torque_limit_[i];
				printMessage("WARNING : desired torque of joint ["+std::to_string(i)+"] exceeds limit\n");
			}
		}

		// Jerk limit
		for (int i=0; i<LBRState::NUMBER_OF_JOINTS; ++i)
		{
			if (desired_torque_[i] - prev_torque_[i] > jerk_limit_[i])
			{
				desired_torque_[i] = prev_torque_[i] + jerk_limit_[i];
				printMessage("WARNING : jerk of joint ["+std::to_string(i)+"] exceeds limit\n");
			}
			else if (desired_torque_[i] - prev_torque_[i] < -jerk_limit_[i])
			{
				desired_torque_[i] = prev_torque_[i] - jerk_limit_[i];
				printMessage("WARNING : jerk of joint ["+std::to_string(i)+"] exceeds limit\n");
			}
		}
		
		// velocity limit
		for (int i=0; i<LBRState::NUMBER_OF_JOINTS; ++i)
		{
			if (vel_raw_(i) > velocity_limit_[i])
			{
				printMessage("WARNING : Velocity of joint ["+std::to_string(i)+"] exceeds limit.\nGoing to fail mode\n");
				exit_program_ = true;
			}
			else if (vel_raw_(i) < -velocity_limit_[i])
			{
				printMessage("WARNING : Velocity of joint ["+std::to_string(i)+"] exceeds limit.\nGoing to fail mode\n");
				exit_program_ = true;
			}
		}

		// Joint Limits
		for (int i=0; i<LBRState::NUMBER_OF_JOINTS; ++i)
		{
			if (position_[i] > joint_limit_[i] - 10.0/180.0*M_PI)
			{
				printMessage("WARNING : Joint ["+std::to_string(i)+"] exceeds soft limit.\nGoing to fail mode\n");
				exit_program_ = true;
			}
			else if (position_[i] < -joint_limit_[i] + 10.0/180.0*M_PI)
			{
				printMessage("WARNING : Joint ["+std::to_string(i)+"] exceeds soft limit.\nGoing to fail mode\n");
				exit_program_ = true;
			}
		}

		// avoid to go too low or too high
		#ifdef USE_KUKA_LBR_DYNAMICS

		Eigen::VectorXd joint_pos_eigen_vector = Eigen::VectorXd::Zero(7);
		for(int i=0; i<7; i++)
		{
			joint_pos_eigen_vector(i) = position_[i];
		}
		wrist_position_ = CalcBodyToBaseCoordinates(rbdl_model_, joint_pos_eigen_vector, 6, Eigen::Vector3d(0.0, 0.0, 0.0), true); // compute the joint position
		
		if(wrist_position_(2) <= lowest_wrist_pos_)
		{
			printMessage("WARNING : Wrist too low.\nGoing to fail mode\n");
			exit_program_ = true;
		}
		if(wrist_position_(2) >= highest_wrist_pos_)
		{
			printMessage("WARNING : Wrist too high.\nGoing to fail mode\n");
			exit_program_ = true;
		}
		#endif


		break;
	}

	// if(counter_%100 == 0)
	// {
	// std::cout << "desired torque suposedly sent to robot (after G compensation + safety) : ";
	// for(int i=0; i<7; i++)
	// {
	// 	std::cout << " " << desired_torque_[i];
	// }
	// std::cout << std::endl;
	// }

	memcpy(prev_torque_, desired_torque_, LBRState::NUMBER_OF_JOINTS * sizeof(double));


	// *-*****************************************************************
	// DEBUG *************************************************************
	// *-*****************************************************************

	// // Send zero torques for debug purpose (except for gravity)
	// for(int i=0; i<7; i++)
	// {
	// 	desired_torque_[i] = 0;
	// }

	// // ADD GRAVITY COMPENSATION FOR TOOL
	// #ifdef USE_KUKA_LBR_DYNAMICS
	// if (fri_command_mode==KUKA::FRI::TORQUE)
	// {
	// 	Eigen::VectorXd q(LBRState::NUMBER_OF_JOINTS);
	// 	Eigen::VectorXd dq(LBRState::NUMBER_OF_JOINTS);
	// 	for (int i=0; i<LBRState::NUMBER_OF_JOINTS; ++i)
	// 	{
	// 		q(i) = position_[i];
	// 		dq(i) = vel_raw_[i];
	// 	}
	// 	Eigen::MatrixXd J(6, LBRState::NUMBER_OF_JOINTS); J.setZero();
	// 	dynamics_.getJacobian(J, q, tool_com_, 7);
	// 	Eigen::VectorXd tool_g_force(6); tool_g_force << 0,0,-9.81*tool_mass_,0,0,0;
	// 	Eigen::VectorXd tool_g_torque = J.transpose()*tool_g_force;
	// 	for (int i=0; i<LBRState::NUMBER_OF_JOINTS; ++i)
	// 	{
	// 		desired_torque_[i] -= tool_g_torque(i);
	// 	}
	// }
	// #endif

	// // COMPENSATE FOR THE OFFSET IN THE FIRST JOINT
	// if (fri_command_mode==KUKA::FRI::TORQUE)
	// {
	// 	desired_torque_[0] -= 0.3;
	// }


	// *-*****************************************************************
	// END DEBUG *************************************************************
	// *-*****************************************************************


	// SEND COMMAND
	switch (fri_command_mode)
	{
		case KUKA::FRI::NO_COMMAND_MODE:
		break;

		case KUKA::FRI::POSITION:
		robotCommand().setJointPosition(desired_position_);
		break;

		case KUKA::FRI::WRENCH:
		printMessage("WARNING : Kuka FRI Wrench control mode has not been implemented in driver");
		break;

		case KUKA::FRI::TORQUE:
		memcpy(dither_, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS*sizeof(double)); 
		for (int i=0; i<7; i++)
		{
			dither_[i] += 0.1/180.0* M_PI*sin(2*M_PI*5*currentTime_);
		}

		// need to command the position even in torque mode in order to activate the friction compensation
		robotCommand().setJointPosition(dither_);
		robotCommand().setTorque(desired_torque_);
		break;

		default:
		printMessage("WARNING : Kuka FRI control mode unrecognized");
		break;
	}

	// update the previous values
	currentTime_ += sampleTime_;

	prev_time_ = time;
	memcpy(prev_position_, position_, LBRState::NUMBER_OF_JOINTS * sizeof(double));
	// memcpy(prev_torque_, desired_torque_, LBRState::NUMBER_OF_JOINTS * sizeof(double));

	counter_++;
}


void LBRRedisClient::printCommandMode(KUKA::FRI::EClientCommandMode command_mode)
{
	switch (command_mode)
	{
		case KUKA::FRI::NO_COMMAND_MODE:
		printMessage("FRI command mode is [NO_COMMAND_MODE]\n");
		break;
		case KUKA::FRI::POSITION:
		printMessage("FRI command mode is [POSITION]\n");
		break;
		case KUKA::FRI::WRENCH:
		printMessage("FRI command mode is [WRENCH]\n");
		break;
		case KUKA::FRI::TORQUE:
		printMessage("FRI command mode is [TORQUE]\n");
		break;
	}
}

void LBRRedisClient::printMessage(const std::string& message)
{
	std::cout << "IIWA Redis Driver : " << message << std::endl;
}

double LBRRedisClient::elapsedTime(timespec start, timespec end)
{
	return (end.tv_sec - start.tv_sec) + 1e-9*(end.tv_nsec - start.tv_nsec);
}


#ifdef USE_KUKA_LBR_DYNAMICS

void LBRRedisClient::parseTool()
{
	tinyxml2::XMLDocument doc;
	doc.LoadFile(tool_file_name_.c_str());
	if (!doc.Error())
	{
		printMessage("Loading tool file ["+tool_file_name_+"].");
		try 
		{
			std::string mass = doc.FirstChildElement("tool")->
			FirstChildElement("inertial")->
			FirstChildElement("mass")->
			Attribute("value");
			tool_mass_ = std::stod(mass);
			printMessage("Tool mass: "+mass);

			std::stringstream com( doc.FirstChildElement("tool")->
				FirstChildElement("inertial")->
				FirstChildElement("origin")->
				Attribute("xyz"));
			com >> tool_com_(0);
			com >> tool_com_(1);
			com >> tool_com_(2);
			std::stringstream ss; ss << tool_com_.transpose();
			printMessage("Tool CoM : "+ss.str());
		}
		catch( const std::exception& e ) // reference to the base of a polymorphic object
		{ 
			std::cout << e.what(); // information from length_error printed
			printMessage("WARNING : Failed to parse tool file.");
		}
	} 
	else 
	{
		printMessage("WARNING : Could no load tool file ["+tool_file_name_+"]");
		doc.PrintError();
	}
}

#endif


} //namespace FRI
} //namespace KUKA
