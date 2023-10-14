#ifndef PAPILLARRAY_ROS2_NODE_H_
#define PAPILLARRAY_ROS2_NODE_H_

#include <stdio.h>

#include <vector>

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

// Messages
#include <papillarray_ros2/msg/pillar_state.hpp>
#include <papillarray_ros2/msg/sensor_state.hpp>
#include "std_msgs/msg/string.hpp"

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

class PapillArrayNode : public rclcpp::Node
{
public:
	// Constructor
	// PapillArrayNode(rclcpp::NodeHandle& nh);
	PapillArrayNode();

	// Destructor
	~PapillArrayNode();

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
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_pubs_; // This one works
	// rclcpp::Publisher<std::vector::vector>::SharedPtr sensor_pubs_; // This is the style I want to work

	// Services
	rclcpp::Service<papillarray_ros2::srv::StartSlipDetection>::SharedPtr start_sd_srv_;
	rclcpp::Service<papillarray_ros2::srv::StopSlipDetection>::SharedPtr stop_sd_srv_;
	rclcpp::Service<papillarray_ros2::srv::BiasRequest>::SharedPtr send_bias_request_srv_;

	// // Service callback functions
	// bool startSlipDetectionSrvCallback(const std::shared_ptr<papillarray_ros2::srv::StartSlipDetection::Request> req,
	// 				std::shared_ptr<papillarray_ros2::srv::StartSlipDetection::Response> resp);
	// bool stopSlipDetectionSrvCallback(const std::shared_ptr<papillarray_ros2::srv::StopSlipDetection::Request> req,
	// 				std::shared_ptr<papillarray_ros2::srv::StopSlipDetection::Response> resp);
	// bool sendBiasRequestSrvCallback(const std::shared_ptr<papillarray_ros2::srv::BiasRequest::Request> req,
	// 				std::shared_ptr<papillarray_ros2::srv::BiasRequest::Response> resp);

	// Load parameters from launch file
	void loadParams();
};

#endif // PAPILLARRAY_ROS_NODE_H_
