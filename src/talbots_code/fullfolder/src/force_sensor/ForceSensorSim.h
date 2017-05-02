// ForceSensorSim.h
// Force sensor for SAI2-Simulation
#ifndef FORCE_SENSOR_SIM_H
#define FORCE_SENSOR_SIM_H

#include "model/ModelInterface.h"
#include "simulation/Sai2Simulation.h"
#include <Eigen/Dense>
#include <string>
#include <vector>

// Basic data structure for force sensor data
struct ForceSensorData {
public:
	// TODO: should probably add some sensor identity as well
	std::string _robot_name; // name of robot to which sensor is attached
	std::string _link_name; // name of link to which sensor is attached
	// transform from link to sensor frame. Measured moments are with respect to
	// the sensor frame origin
	Eigen::Affine3d _transform_in_link;
	Eigen::Vector3d _force; // in sensor frame
	Eigen::Vector3d _moment; // in sensor frame

public:
	// ctor: assign defaults
	ForceSensorData()
	: _robot_name(""),
	_link_name(""),
	_transform_in_link(Eigen::Affine3d::Identity()),
	_force(Eigen::Vector3d::Zero()),
	_moment(Eigen::Vector3d::Zero())
	{
		// nothing to do
	}
};

// Simulated force sensor type.
// Note that this implementation ignores the mass and inertia of the object
// attached beyond the force sensor.
// Note also that this assumes the force sensor to be located just between the
// link and the end-effector. That is, it cannot account for any joint reaction
// forces in case the sensor is to be attached on the link between two joints.
// Finally note that the sensor returns values in the global frame
class ForceSensorSim {
public:
	// ctor
	ForceSensorSim(
		const std::string& robot_name,
		const std::string& link_name,
		const Eigen::Affine3d& transform_in_link,
		Simulation::Sai2Simulation* sim,
		Model::ModelInterface* model)
	: _sim(sim),
	_model(model)
	{
		_data = new ForceSensorData();
		_data->_robot_name = robot_name;
		_data->_link_name = link_name;
		_data->_transform_in_link = transform_in_link;
	}

	//dtor
	~ForceSensorSim() {
		delete _sim;
	}

	// update force information
	void update() {
		// NOTE that this assumes that the robot model is updated
		// get the list of contact forces acting on the link
		std::vector<Eigen::Vector3d> force_list;
		std::vector<Eigen::Vector3d> point_list;
		_sim->getContactList(
			point_list,
			force_list,
			_data->_robot_name,
			_data->_link_name);
		// zero out current forces and moments
		_data->_force.setZero();
		_data->_moment.setZero();
		// if list is empty, simply set forces and moments to 0
		if(point_list.empty()) {
			return;
		}
		// transform to sensor frame
		Eigen::Vector3d rel_pos;
		Eigen::Vector3d link_pos;
		_model->position(link_pos, _data->_link_name, _data->_transform_in_link.translation());
		for (uint pt_ind=0; pt_ind < point_list.size(); ++pt_ind) {
			_data->_force += force_list[pt_ind];
			rel_pos = point_list[pt_ind] - link_pos;
			//unfortunately, it is defined in global frame
			_data->_moment += rel_pos.cross(force_list[pt_ind]);
		}
		_data->_force = _data->_transform_in_link.inverse().rotation()*_data->_force;
		_data->_moment = _data->_transform_in_link.inverse().rotation()*_data->_moment;
	}

	// get force
	void getForce(Eigen::Vector3d& ret_force) {
		ret_force = _data->_force;
	}

	// get moment
	void getMoment(Eigen::Vector3d& ret_moment) {
		ret_moment = _data->_moment;
	}

public:
	// handle to simulation interface
	Simulation::Sai2Simulation* _sim;

	// handle to model interface
	Model::ModelInterface* _model;

	// last updated data
	ForceSensorData* _data;
};

#endif //FORCE_SENSOR_SIM_H
