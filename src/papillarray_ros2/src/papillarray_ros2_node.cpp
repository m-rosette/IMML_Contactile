#include <papillarray_ros2_node.hpp>

PapillArrayNode::PapillArrayNode()
: Node("papillarray_ros2"), count(0)
{
	// rclcpp::NodeHandle& nh) : listener_(true) { // listener_ argument: isLogging to .csv file; Log file written to /home/.ros/Logs

	RCLCPP_INFO("Loading parameters...\n");
	loadParams(nh);

	// Create sensors and add to listener
	RCLCPP_INFO("Creating sensors...\n");

	for (int i = 0; i < n_sensors_; i++) {
		RCLCPP_INFO("Creating sensor %d...", i);
		auto sensor = std::make_unique<PTSDKSensor>();
		RCLCPP_INFO("Adding sensor %d to listener...", i);
		listener_.addSensor(sensor.get());
		RCLCPP_INFO("Added sensor %d to listener!\n", i);
		sensors_.push_back(std::move(sensor));

		// Setup publisher for sensor
		std::string topic = "/hub_" + std::to_string(hub_id_) + "/sensor_" + std::to_string(i);
		rclcpp::Publisher sensor_pub = nh.advertise<papillarray_ros2::SensorState>(topic, 1);
		sensor_pubs_.push_back(sensor_pub);
	}

	// Start services
	RCLCPP_INFO("Starting services...");
	std::string service_name = "/hub_" + std::to_string(hub_id_) + "/start_slip_detection";
	start_sd_srv_ = nh.advertiseService(service_name,&PapillArrayNode::startSlipDetectionSrvCallback,this);
	if (rclcpp::service::exists(service_name, true)) {
		RCLCPP_INFO("Started %s service", service_name.c_str());
	} else {
		RCLCPP_WARN("%s service not advertised", service_name.c_str());
	}

	service_name = "/hub_" + std::to_string(hub_id_) + "/stop_slip_detection";
	stop_sd_srv_ = nh.advertiseService(service_name,&PapillArrayNode::stopSlipDetectionSrvCallback,this);
	if (rclcpp::service::exists(service_name, true)) {
		RCLCPP_INFO("Started %s service", service_name.c_str());
	} else {
		RCLCPP_WARN("%s service not advertised", service_name.c_str());
	}

	service_name = "/hub_" + std::to_string(hub_id_) + "/send_bias_request";
	send_bias_request_srv_ = nh.advertiseService(service_name,&PapillArrayNode::sendBiasRequestSrvCallback,this);
	if (rclcpp::service::exists(service_name, true)) {
		RCLCPP_INFO("Started %s service", service_name.c_str());
	} else {
		RCLCPP_WARN("%s service not advertised", service_name.c_str());
	}

	// Start listener
	RCLCPP_INFO("Connecting to %s port...", port_.c_str());
	if (listener_.connectAndStartListening(port_.c_str(), baud_rate_, parity_, char(byte_size_), is_flush_)) {
		RCLCPP_FATAL("Failed to connect to port: %s", port_.c_str());
	} else {
		RCLCPP_INFO("Connected to port: %s", port_.c_str());
	}

	// Set sampling rate
	RCLCPP_INFO("Setting sampling rate to %u...",sampling_rate_);
	if (!listener_.setSamplingRate(sampling_rate_)) {
		RCLCPP_WARN("Failed to set sampling rate to: %u", sampling_rate_);
	} else {
		RCLCPP_INFO("Sampling rate set to %u", sampling_rate_);
	}
}

void PapillArrayNode::loadParams(rclcpp::Node("papillarray_ros2")) {
	if (nh.getParam("hub_id", hub_id_)) {
		RCLCPP_INFO("Hub id: %d", hub_id_);
	} else {
		RCLCPP_ERROR("Failed to read hub id param. Ensure set correctly in launch file");
	}
	if (nh.getParam("n_sensors", n_sensors_)) {
		if (n_sensors_ > MAX_NSENSOR or n_sensors_ < 1) {
			RCLCPP_ERROR("Invalid number of sensors!  %d selected (must be no more than %d)", n_sensors_, MAX_NSENSOR);
		} else {
			RCLCPP_INFO("Using %d sensor/s", n_sensors_);
		}
	} else {
		RCLCPP_ERROR("Failed to read number_of_sensors param.  Ensure set correctly in launch file");
	}
	if (nh.getParam("com_port", port_)) {
		RCLCPP_INFO("Reading from serial COM port: %s", port_.c_str());
	} else {
		RCLCPP_ERROR("Failed to read COM port param. Ensure set correctly in launch file");
	}
	if (nh.getParam("baud_rate", baud_rate_)) {
		RCLCPP_INFO("Baud rate: %d Hz", baud_rate_);
	} else {
		RCLCPP_ERROR("Failed to read baud_rate param.  Ensure set correctly in launch file");
	}
	if (nh.getParam("parity", parity_)) {
		RCLCPP_INFO("Parity set to: %d (0=PARITY_NONE, 1=PARITY_ODD, 2=PARITY_EVEN)", parity_);
	} else {
		RCLCPP_ERROR("Failed to read paraty param.  Ensure set correctly in launch file");
	}
	if (nh.getParam("byte_size", byte_size_)) {
		RCLCPP_INFO("Byte size: %d bits", byte_size_);
	} else {
		RCLCPP_ERROR("Failed to read byte_size param.  Ensure set correctly in launch file");
	}

	if (nh.getParam("is_flush", is_flush_)) {
		RCLCPP_INFO("Is Flush: %d", is_flush_);
	} else {
		RCLCPP_ERROR("Failed to read is_flush param. Ensure set correctly in launch file");
	}

	if (nh.getParam("sampling_rate", sampling_rate_)) {
		RCLCPP_INFO("Sampling rate: %d Hz", sampling_rate_);
	} else {
		RCLCPP_ERROR("Failed to read sampling_rate param. Ensure set correctly in launch file");
	}

	RCLCPP_INFO("Loaded parameters.\n");
}

void PapillArrayNode::updateData() {
	if (n_sensors_ == 0) {
		return;
	}

	for (int sensor_id = 0; sensor_id < sensors_.size(); sensor_id++) {
		papillarray_ros2::msg::SensorState ss_msg;

		//RCLCPP_INFO("N pillars: %d", sensors_[sensor_id]->getNPillar());
		std_msgs::msg::Header h;
		auto time = rclcpp::Time::now();
		h.stamp.sec = time.sec;
		h.stamp.nsec = time.nsec;
		ss_msg.header = h;

		long timestamp_us = sensors_[sensor_id]->getTimestamp_us();
		ss_msg.tus = timestamp_us;

		double globalForce[NDIM];
		double globalTorque[NDIM];

		// Read global forces
		sensors_[sensor_id]->getGlobalForce(globalForce);
		ss_msg.gfx = static_cast<float>(globalForce[X_IND]);
		ss_msg.gfy = static_cast<float>(globalForce[Y_IND]);
		ss_msg.gfz = static_cast<float>(globalForce[Z_IND]);
		// Read global torques
		sensors_[sensor_id]->getGlobalTorque(globalTorque);
		ss_msg.gtx = static_cast<float>(globalTorque[X_IND]);
		ss_msg.gty = static_cast<float>(globalTorque[Y_IND]);
		ss_msg.gtz = static_cast<float>(globalTorque[Z_IND]);

		// Friction estimate
		ss_msg.friction_est = static_cast<float>(sensors_[sensor_id]->getFrictionEstimate());

		// Target grip force (-1 if no friction estimate)
		ss_msg.target_grip_force = static_cast<float>(sensors_[sensor_id]->getTargetGripForce());

		int n_pillar = sensors_[sensor_id]->getNPillar();

		bool is_sd_active;
		bool is_ref_loaded;
		bool contact_states[n_pillar];
		int slip_states[n_pillar];

		sensors_[sensor_id]->getAllSlipStatus(&is_sd_active,
								&is_ref_loaded,
								contact_states,
								slip_states);

		ss_msg.is_sd_active = is_sd_active;
		ss_msg.is_ref_loaded = is_ref_loaded;
		ss_msg.is_contact = false;

		// Get PillarState data for all pillars in sensor array
		for (int pillar_id = 0; pillar_id < n_pillar; pillar_id++) {
			papillarray_ros2::msg::PillarState ps_msg;

			ps_msg.id = pillar_id;
			ps_msg.slip_state = slip_states[pillar_id];
			ps_msg.in_contact = contact_states[pillar_id];

			ss_msg.is_contact = ss_msg.is_contact | ps_msg.in_contact;

			double pillar_d[NDIM];
			sensors_[sensor_id]->getPillarDisplacements(pillar_id, pillar_d);
			ps_msg.dx = static_cast<float>(pillar_d[X_IND]);
			ps_msg.dy = static_cast<float>(pillar_d[Y_IND]);
			ps_msg.dz = static_cast<float>(pillar_d[Z_IND]);

			double pillar_f[NDIM];
			sensors_[sensor_id]->getPillarForces(pillar_id, pillar_f);
			ps_msg.fx = static_cast<float>(pillar_f[X_IND]);
			ps_msg.fy = static_cast<float>(pillar_f[Y_IND]);
			ps_msg.fz = static_cast<float>(pillar_f[Z_IND]);

			ss_msg.pillars.push_back(ps_msg);

			if(pillar_id == 0){
				//RCLCPP_INFO("From C++API: %.2f; From ROS: %.2f\n",pillar_f[2], ps_msg.fZ);
			}
		}

		// Publish SensorState message
		sensor_pubs_[sensor_id].publish(ss_msg);
	}
}

bool PapillArrayNode::startSlipDetectionSrvCallback(papillarray_ros2::StartSlipDetection::Request &req,
						papillarray_ros2::StartSlipDetection::Response &resp) {
	RCLCPP_INFO("startSlipDetection callback");
	resp.result = listener_.startSlipDetection();
	rclcpp::Duration(0.1).sleep(); // wait
	return resp.result;
}

bool PapillArrayNode::stopSlipDetectionSrvCallback(papillarray_ros2::StopSlipDetection::Request &req,
						   papillarray_ros2::StopSlipDetection::Response &resp) {
	RCLCPP_INFO("stopSlipDetection callback");
	resp.result = listener_.stopSlipDetection();
	rclcpp::Duration(0.1).sleep(); // wait
	return resp.result;
}

bool PapillArrayNode::sendBiasRequestSrvCallback(papillarray_ros2::BiasRequest::Request &req,
						 papillarray_ros2::BiasRequest::Response &resp) {
	RCLCPP_INFO("sendBiasRequest callback");
	resp.result = listener_.sendBiasRequest();
	rclcpp::Duration(0.1).sleep(); // wait
	return resp.result;
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv, "papillarray_ros2");

	rclcpp::NodeHandle nh("~");

	PapillArrayNode papill_array_node(nh);

	rclcpp::Rate loop_rate(papill_array_node.getSamplingRate());
	loop_rate.sleep();

	int i = 0;
	while (rclcpp::ok()) {
		rclcpp::spin_some(PapillArrayNode);
		loop_rate.sleep();
		papill_array_node.updateData();	// Update sensor data and publish
		//RCLCPP_INFO("Still spinning: %d",i);
		i++;
	}

	return 0;
}
