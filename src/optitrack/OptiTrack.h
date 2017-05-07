#ifndef OPTITRACK_H
#define OPTITRACK_H

// NatNetLinux
#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>

// std
#include <vector>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

template <int Fps, int NoDropFps>
class OptiTrack;

typedef OptiTrack<120, 30> OptiTrack225a;

template <int CameraFps, int ExportFps>
class OptiTrack {

public:

	OptiTrack() :
		t_frame_(0),
		t_frame_next_(0)
	{};
	~OptiTrack();

	typedef std::chrono::duration<int, std::ratio<1, CameraFps>> SubFrameTime;
	typedef std::chrono::duration<int, std::ratio<1, ExportFps>> FrameTime;

	bool openConnection(const std::string& local_ip_address, const std::string& server_ip_address = "172.24.68.48");
	void closeConnection();

	bool openCsv(const std::string& filename);
	void closeCsv();

	bool getFrame();

	std::vector<Eigen::Vector3f> pos_rigid_bodies_;
	std::vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Quaternionf>> ori_rigid_bodies_;
	std::vector<Eigen::Vector3f> pos_single_markers_;

	SubFrameTime t_frame_;

private:

	enum MarkerType {
		RIGID_BODY_POSITION,
		RIGID_BODY_ORIENTATION,
		SINGLE_MARKER_POSITION,
		OTHER
	};

	bool readNetworkFrame();
	bool readCsvFrame();

	// Listener threads
	std::unique_ptr<CommandListener> command_listener_;
	std::unique_ptr<FrameListener> frame_listener_;

	// Socket descriptors
	int sd_data_;
	int sd_command_;

	// CSV variables
	std::ifstream csv_file_;
	std::vector<MarkerType> marker_types_;

	// Timer
	bool t_initialized_ = false;
	std::chrono::high_resolution_clock::time_point t_start_;
	FrameTime t_frame_next_;

};

/*** NOTE: Template definitions need to be placed in header file. ***/

template <int F1, int F2>
OptiTrack<F1,F2>::~OptiTrack(){
	closeConnection();
	closeCsv();
};

template <int F1, int F2>
bool OptiTrack<F1,F2>::openConnection(const std::string& local_ip_address, const std::string& server_ip_address) {
    uint32_t local_address = inet_addr(local_ip_address.c_str());
    uint32_t server_address = inet_addr(server_ip_address.c_str());

	// Use this socket address to send commands to the server.
	struct sockaddr_in server_commands = NatNet::createAddress(server_address, NatNet::commandPort);

	// Create sockets
	sd_command_ = NatNet::createCommandSocket(local_address);
	sd_data_ = NatNet::createDataSocket(local_address);

	// Start the CommandListener in a new thread.
	command_listener_ = std::make_unique<CommandListener>(sd_command_);
	command_listener_->start();

	// Send a ping packet to the server so that it sends us the NatNet version
	// in its response to commandListener.
	NatNetPacket ping = NatNetPacket::pingPacket();
	ssize_t status = ping.send(sd_command_, server_commands);

	// Wait here for ping response to give us the NatNet version.
	unsigned char natnet_major;
	unsigned char natnet_minor;
	bool success = command_listener_->getNatNetVersion(natnet_major, natnet_minor);
	if (!success) {
		std::cout << "WARNING: Failed to connect to OptiTrack server " << server_ip_address << "." << std::endl << std::endl;
		command_listener_->stop();
		command_listener_->join();
		command_listener_.release();
		close(sd_command_);
		close(sd_data_);
		return false;
	}

	// Start up a FrameListener in a new thread.
	frame_listener_ = std::make_unique<FrameListener>(sd_data_, natnet_major, natnet_minor);
	frame_listener_->start();
	return true;
}

template <int F1, int F2>
void OptiTrack<F1,F2>::closeConnection() {
	// Wait for threads to finish
	if (command_listener_) command_listener_->stop();
	if (frame_listener_) frame_listener_->stop();
	if (command_listener_) command_listener_->join();
	if (frame_listener_) frame_listener_->join();

	// Close sockets
	close(sd_command_);
	close(sd_data_);
}

template <int F1, int F2>
bool OptiTrack<F1,F2>::openCsv(const std::string& filename) {
	csv_file_.open(filename);
	if (csv_file_.fail()) return false;

	int num_rigid_bodies = 0;
	int num_markers = 0;

	std::string line, cell;
	std::stringstream ss_line;
	std::getline(csv_file_, line);  // First line
	std::getline(csv_file_, line);  // Empty line

	std::getline(csv_file_, line);  // Data type
	ss_line.str(line);
	std::getline(ss_line, cell, ',');  // Frame
	std::getline(ss_line, cell, ',');  // Time
	while (std::getline(ss_line, cell, ',')) {
		if (cell == "Rigid Body") {
			marker_types_.push_back(RIGID_BODY_POSITION);
			marker_types_.push_back(RIGID_BODY_POSITION);
			// Skip -YZWXY
			for (int i = 0; i < 6; i++) std::getline(ss_line, cell, ',');
			num_rigid_bodies++;
		} else if (cell == "Marker") {
			marker_types_.push_back(SINGLE_MARKER_POSITION);
			// Skip -YZ
			for (int i = 0; i < 2; i++) std::getline(ss_line, cell, ',');
			num_markers++;
		}
	}

	std::getline(csv_file_, line);  // Label
	std::getline(csv_file_, line);  // Marker ID

	std::getline(csv_file_, line);  // Position/Orientation
	ss_line.str(line);
	std::getline(ss_line, cell, ',');  // Frame
	std::getline(ss_line, cell, ',');  // Time
	int idx = 0;
	while (std::getline(ss_line, cell, ',')) {
		if (cell == "Rotation") {
			marker_types_[idx] = RIGID_BODY_ORIENTATION;
			// Skip -YZW
			for (int i = 0; i < 3; i++) std::getline(ss_line, cell, ',');
		} else {
			// Skip -YZ
			for (int i = 0; i < 2; i++) std::getline(ss_line, cell, ',');
		}
		idx++;
	}

	std::getline(csv_file_, line);  // XYZW

	// Set buffer sizes
	pos_rigid_bodies_.resize(num_rigid_bodies);
	ori_rigid_bodies_.resize(num_rigid_bodies);
	pos_single_markers_.resize(num_markers);

	return true;
}

template <int F1, int F2>
void OptiTrack<F1,F2>::closeCsv() {
	csv_file_.close();
}

template <int F1, int F2>
bool OptiTrack<F1,F2>::getFrame() {
	if (csv_file_) return readCsvFrame();
	if (command_listener_ && frame_listener_) return readNetworkFrame();
	std::cout << "OptiTrack::getFrame(): No open connection or specified csv file." << std::endl;
	return false;
}

template <int F1, int F2>
bool OptiTrack<F1,F2>::readNetworkFrame() {
	// Try to get a new frame from the listener.
	bool frame_available;
	MocapFrame frame(frame_listener_->pop(&frame_available).first);
	if (!frame_available) return false;

	// Get recorded time
	int hour, min, sec, fframe, subframe;
	frame.timecode(hour, min, sec, fframe, subframe);

	std::chrono::hours t_hour(hour);
	std::chrono::minutes t_min(min);
	std::chrono::seconds t_sec(sec);
	FrameTime t_frame(fframe);
	SubFrameTime t_subframe(subframe);
	t_frame_ = t_hour + t_min + t_sec + t_frame + t_subframe;

	// Get rigid bodies
	std::vector<RigidBody> rigid_bodies = frame.rigidBodies();
	pos_rigid_bodies_.resize(rigid_bodies.size());
	ori_rigid_bodies_.resize(rigid_bodies.size());

	for (size_t i = 0; i < rigid_bodies.size(); i++) {
		Point3f pos = rigid_bodies[i].location();
		Quaternion4f ori = rigid_bodies[i].orientation();
		pos_rigid_bodies_[i] = Eigen::Vector3f(pos.x, pos.y, pos.z);
		ori_rigid_bodies_[i] = Eigen::Quaternionf(ori.qw, ori.qx, ori.qy, ori.qz);
	}

	// Get unidentified markers
	std::vector<Point3f> single_markers = frame.unIdMarkers();
	pos_single_markers_.resize(single_markers.size());

	for (size_t i = 0; i < rigid_bodies.size(); i++) {
		Point3f pos = single_markers[i];
		pos_single_markers_[i] = Eigen::Vector3f(pos.x, pos.y, pos.z);
	}

	return true;
}

template <int F1, int F2>
bool OptiTrack<F1,F2>::readCsvFrame() {
	// Check time
	if (!t_initialized_) {
		// Initialize time and continue
		t_start_ = std::chrono::high_resolution_clock::now();
		t_initialized_ = true;
	} else {
		// Calculate current frame time
		auto t_curr = std::chrono::high_resolution_clock::now();
		FrameTime t_frame_curr = std::chrono::duration_cast<FrameTime>(t_curr - t_start_);
		int num_frames_diff = t_frame_curr.count() - t_frame_next_.count();

		// Return if not yet time for next frame
		if (num_frames_diff < 0) return false;

		// Set next frame time
		t_frame_next_ += FrameTime(1 + num_frames_diff);

		// Fast forward to most recent frame
		while (num_frames_diff > 0) {
			std::string line;
			std::getline(csv_file_, line);
			num_frames_diff--;
		}
	}

	// Read line
	std::string line;
	if (!std::getline(csv_file_, line)) return false;
	std::stringstream ss_line(line);

	// Parse frame number and time in seconds
	int num_frame;
	double sec_frame;
	char comma;
	ss_line >> num_frame >> comma >> sec_frame;
	t_frame_ += FrameTime(1);

	// Parse frame
	int idx_pos_rigid_body = 0;
	int idx_ori_rigid_body = 0;
	int idx_pos_single_marker = 0;
	for (auto marker_type : marker_types_) {
		if (marker_type == RIGID_BODY_POSITION) {
			// Parse rigid body position: x,y,z
			float x, y, z;
			ss_line >> x >> comma >> y >> comma >> z;
			pos_rigid_bodies_[idx_pos_rigid_body] = Eigen::Vector3f(x, y, z);
			idx_pos_rigid_body++;
		} else if (marker_type == RIGID_BODY_ORIENTATION) {
			// Parse rigid body orientation: x,y,z,w
			float w, x, y, z;
			ss_line >> x >> comma >> y >> comma >> z >> comma >> w;
			ori_rigid_bodies_[idx_ori_rigid_body] = Eigen::Quaternionf(w, x, y, z);
			idx_ori_rigid_body++;
		} else if (marker_type == SINGLE_MARKER_POSITION) {
			// Parse single marker position: x,y,z
			float x, y, z;
			ss_line >> x >> comma >> y >> comma >> z;
			pos_single_markers_[idx_pos_single_marker] = Eigen::Vector3f(x, y, z);
			idx_pos_single_marker++;
		}
		// Parse comma
		ss_line >> comma;
	}

	return true;
}

#endif  // OPTITRACK_H
