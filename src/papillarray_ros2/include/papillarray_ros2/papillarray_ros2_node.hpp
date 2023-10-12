#ifndef PAPILLARRAY_ROS_V2_NODE_H_
#define PAPILLARRAY_ROS_V2_NODE_H_

#include <stdio.h>

#include <vector>

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

// Messages
#include <papillarray_ros2/msg/pillar_state.hpp>
#include <papillarray_ros2/msg/sensor_state.hpp>

// Services
#include <papillarray_ros2/srv/start_slip_detection.hpp>
#include <papillarray_ros2/srv/stop_slip_detection.hpp>
#include <papillarray_ros2/srv/bias_request.hpp>
//#include <papillarray_ros2/SetSamplingRate.h>

#ifdef __unix__
typedef unsigned char byte;
#endif

#ifndef PTSDKCONSTANTS_H
#include <PTSDKConstants.h>
#endif
#ifndef PTSDKLISTENER_H
#include <PTSDKListener.h>
#endif
#ifndef PTSDKSENSOR_H
#include <PTSDKSensor.h>
#endif

class PapillArrayNode : rclcpp::Node
{
public:
	// Constructor
	// PapillArrayNode(rclcpp::NodeHandle& nh);
	PapillArrayNode() : Node("papillarray_ros2")

	// Destructor
	~PapillArrayNode() {
		// Stop listening for and processing data and disconnect from the COM port
		listener_.stopListeningAndDisconnect();
	}

	// Update sensor data and publish
	void updateData();

	// Get the sampling rate
	int getSamplingRate(){ return sampling_rate_; };

private:
	int hub_id_;

	int n_sensors_;

	std::string port_;
	int baud_rate_;
	int parity_;
	int byte_size_;
	bool is_flush_;
	int sampling_rate_;

	PTSDKListener listener_;
	std::vector<std::unique_ptr<PTSDKSensor> > sensors_;

	// Sensor publishers
	std::vector<rclcpp::Publisher> sensor_pubs_;

	// Services
	rclcpp::ServiceServer start_sd_srv_;
	rclcpp::ServiceServer stop_sd_srv_;
	rclcpp::ServiceServer send_bias_request_srv_;

	// Service callback functions
	bool startSlipDetectionSrvCallback(papillarray_ros2::StartSlipDetection::Request &req,
					papillarray_ros2::StartSlipDetection::Response &resp);
	bool stopSlipDetectionSrvCallback(papillarray_ros2::StopSlipDetection::Request &req,
					papillarray_ros2::StopSlipDetection::Response &resp);
	bool sendBiasRequestSrvCallback(papillarray_ros2::BiasRequest::Request &req,
					papillarray_ros2::BiasRequest::Response &resp);

	// Load parameters from launch file
	void loadParams(rclcpp::NodeHandle& nh);
};

#endif // PAPILLARRAY_ROS_NODE_H_
