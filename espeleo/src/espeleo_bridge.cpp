/*
 * Copyright (c) 2015-2016, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "bridge.h"
#include "logger.h"
#include "joint_state_publisher.h"
#include "joint_state_subscriber.h"
#include "entry_publisher.h"
#include "entry_subscriber.h"
#include "mapping.h"
#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include "publisher.h"


#include <thread>
#include <chrono>
#include <memory>

// #define BUSNAME ... // set by CMake
// #define BAUDRATE ... // set by CMake

kaco::Master master;
kaco::Bridge bridge;
int acceleration = 10000;
int deceleration = 40000;

bool reset_motors(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
	
	std::vector<std::shared_ptr<kaco::Publisher>> pub_list = bridge.get_publishers();
	for(unsigned int i=0; i < pub_list.size(); ++i){
		pub_list[i]->set_publish_state(false);
	}

	ROS_INFO("resetting CAN communication and nodes");
	master.core.nmt.reset_communication_all_nodes();
	master.core.nmt.reset_all_nodes();

	ROS_INFO("SLEEPING...");
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	ROS_INFO("AWAKE...");

	for (size_t i=0; i < master.num_devices(); ++i) {
		kaco::Device& device = master.get_device(i);
		device.start();
		device.load_dictionary_from_library();

		const kaco::Value& accel((uint32_t)acceleration);
		const kaco::Value& decel((uint32_t)deceleration);
		device.set_entry("profile_acceleration", accel);
		device.set_entry("profile_deceleration", decel);

		const auto profile = device.get_device_profile_number();
		PRINT("Found CiA "<<std::dec<<(unsigned)profile<<" device with node ID "<<device.get_node_id()<<": "<<device.get_entry("manufacturer_device_name"));
		
		if (profile==402) {

			PRINT("Set velocity mode");
			device.set_entry("modes_of_operation", device.get_constant("profile_velocity_mode"));

			// PRINT("Set position mode");
			// device.set_entry("modes_of_operation", device.get_constant("profile_position_mode"));

			PRINT("Enable operation");
			//device.execute("initialise_motor");
			device.execute("enable_operation");
		}
	}

	ROS_INFO("SLEEPING...");
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	ROS_INFO("AWAKE...");

	for(std::size_t i=0; i<pub_list.size(); ++i){
		pub_list[i]->advertise();
		pub_list[i]->set_publish_state(true);
	}

  	res.success = true;
  	ROS_INFO("sending back response: [%d]", res.success);

  	return true;
}

int main(int argc, char* argv[]) {

	// Set the name of your CAN bus. "slcan0" is a common bus name
	// for the first SocketCAN device on a Linux system.
  	const std::string busname = "can0";

	// Set the baudrate of your CAN bus. Most drivers support the values
	// "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
	const std::string baudrate = "1M";

	PRINT("This example publishes and subscribes JointState messages for each connected CiA 402 device as well as"
		<<"uint8 messages for each connected digital IO device (CiA 401).");

	const double loop_rate = 5; // [Hz]

	
	if (!master.start(busname, baudrate)) {
		ERROR("Starting master failed.");
		return EXIT_FAILURE;
	}

	master.core.nmt.reset_communication_all_nodes();
	master.core.nmt.reset_all_nodes();

	std::this_thread::sleep_for(std::chrono::seconds(1));
	size_t num_devices_required = 1;
	while (master.num_devices()<num_devices_required) {
		ERROR("Number of devices found: " << master.num_devices() << ". Waiting for " << num_devices_required << ".");
		PRINT("Trying to discover more nodes via NMT Node Guarding...");
		master.core.nmt.discover_nodes();
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	// Create bridge
	ros::init(argc, argv, "canopen_bridge");

  	ros::NodeHandle n;
  	ros::ServiceServer service = n.advertiseService("reset_motors", reset_motors);

	ros::param::get("~acceleration", acceleration);
	ros::param::get("~deceleration", deceleration);

	bool found = false;
	for (size_t i=0; i<master.num_devices(); ++i) {

		kaco::Device& device = master.get_device(i);
		device.start();

		device.load_dictionary_from_library();

		const kaco::Value& accel((uint32_t)acceleration);
		const kaco::Value& decel((uint32_t)deceleration);
		device.set_entry("profile_acceleration", accel);
		device.set_entry("profile_deceleration", decel);

		const auto profile = device.get_device_profile_number();
		PRINT("Found CiA "<<std::dec<<(unsigned)profile<<" device with node ID "<<device.get_node_id()<<": "<<device.get_entry("manufacturer_device_name"));
		
		if (profile==402) {

			found = true;

			// //PDOs for General Control word (0x200 + Device id)
			// //PDOs for Profile Position (0x300 + Device id)
			// //PDOs for Profile Velocity (0x400 + Device id)
			// //PDOs for Profile Torque (0x500 + Device id)
			// std::vector<kaco::Mapping> mapping_200, mapping_300, mapping_400, mapping_500;
			// kaco::Mapping mapping_cw, mapping_target_pos, mapping_target_vel, mapping_target_torq;
			// mapping_cw.entry_name = "Controlword";
			// mapping_cw.offset = 0;
			// mapping_200.push_back(mapping_cw);
			// mapping_300.push_back(mapping_cw);
			// mapping_400.push_back(mapping_cw);
			// mapping_500.push_back(mapping_cw);
			// mapping_target_pos.entry_name = "Target Position";
			// mapping_target_pos.offset = 2;
			// mapping_300.push_back(mapping_target_pos);
			// mapping_target_vel.entry_name = "Target Velocity";
			// mapping_target_vel.offset = 2;
			// mapping_400.push_back(mapping_target_vel);
			// mapping_target_torq.entry_name = "Target Torque";
			// mapping_target_torq.offset = 2;
			// mapping_500.push_back(mapping_target_torq);
			// device.add_transmit_pdo_mapping(0x27F, mapping_200);
			// //device.add_transmit_pdo_mapping(0x37F, mapping_300);
			// device.add_transmit_pdo_mapping(0x47F, mapping_400);
			// //device.add_transmit_pdo_mapping(0x57F, mapping_500);

			PRINT("Set velocity mode");
			device.set_entry("modes_of_operation", device.get_constant("profile_velocity_mode"));

			// PRINT("Set position mode");
			// device.set_entry("modes_of_operation", device.get_constant("profile_position_mode"));

			PRINT("Enable operation");
			//device.execute("initialise_motor");
			device.execute("enable_operation");

			auto joint_state_pub = std::make_shared<kaco::JointStatePublisher>(device, 0, 350000);
			bridge.add_publisher(joint_state_pub, loop_rate);

			auto status_pub = std::make_shared<kaco::EntryPublisher>(device, "statusword");
			bridge.add_publisher(status_pub, loop_rate);

			auto current_pub = std::make_shared<kaco::EntryPublisher>(device, "current_actual_value");
			bridge.add_publisher(current_pub, loop_rate);

			auto joint_state_sub = std::make_shared<kaco::JointStateSubscriber>(device, 0, 350000);
			bridge.add_subscriber(joint_state_sub);
		}

	}

	if (!found) {
		ERROR("This example is intended for use with a CiA 402 device but I can't find one.");
		return EXIT_FAILURE;
	}

	bridge.run();
}
