/* This code is based on templates provided by Kinova. It has been edited
 * for use with the Stanford Robotics Lab robots.
 *
 * The edits are subject to the following license.
 */

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Copyright Aug 2016, Stanford Robotics Lab, Stanford University
 *
 * Edited by : Toki Migimatsu
 *             Samir Menon
 */

#include <KinovaTypes.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>

#include <iostream>

int main()
{
	std::cout << std::endl
	          << "Kinova gravity parameter calibration." << std::endl
	          << "Please clear the area around the robot. Process may take ~15 min." << std::endl
	          << std::endl;

	// Initialize API
	if (InitAPI() != 1) {
		std::cout << "Error initializing API. Make sure robot is connected." << std::endl;
		return 1;
	}

	// Test communication with robot
	AngularPosition kinova_position;
	if (GetAngularCommand(kinova_position) != 1) {
		std::cout << "Error communicating with robot." << std::endl;
		return 1;
	}

	std::cout << "Initialization complete. Starting parameter estimation sequence..." << std::endl
	          << std::endl;

	// Run parameter estimation sequence
	ROBOT_TYPE robot_type = JACOV2_6DOF_SERVICE;
	double optimal_z[OPTIMAL_Z_PARAM_SIZE];
	RunGravityZEstimationSequence(robot_type, optimal_z);

	std::cout << "Paste the following line into src/kinova_jaco/gravity.xml:" << std::endl
	          << std::endl
	          << "<gravity>" << optimal_z[0];
	for (int i = 1; i < OPTIMAL_Z_PARAM_SIZE; i++) {
		std::cout << " " << optimal_z[i];
	}
	std::cout << "</gravity>" << std::endl
	          << std::endl;

	// Close API
	std::cout << "Closing API." << std::endl;
	CloseAPI();

	return 0;
}
