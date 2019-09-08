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
#include "test_joint_state_publisher.h"
#include "test_joint_state_subscriber.h"
#include "test_entry_publisher.h"
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

bool reset_motors(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
	
	ROS_INFO("RESET MOTORS CALLED");

	ROS_INFO("SLEEPING...");
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	ROS_INFO("AWAKE...");

  	res.success = true;
  	ROS_INFO("sending back response: [%d]", res.success);

  	return true;
}

int main(int argc, char* argv[]) {

	PRINT("NODE FOR TESTING CALLBACK BEHAVIOUR");

	const double loop_rate = 5; // [Hz]

	// Create bridge
	ros::init(argc, argv, "callback_test_problem");

  	ros::NodeHandle n;
  	ros::ServiceServer service = n.advertiseService("reset_motors", reset_motors);

	for (size_t i=0; i < 6; ++i) {

		uint8_t device = i + 1;

		auto joint_state_pub = std::make_shared<kaco::TestJointStatePublisher>(device, 0, 350000);
		bridge.add_publisher(joint_state_pub, loop_rate);

		auto status_pub = std::make_shared<kaco::TestEntryPublisher>(device, "statusword");
		bridge.add_publisher(status_pub, loop_rate);

		auto current_pub = std::make_shared<kaco::TestEntryPublisher>(device, "current_actual_value");
		bridge.add_publisher(current_pub, loop_rate);

		auto joint_state_sub = std::make_shared<kaco::TestJointStateSubscriber>(device, 0, 350000);
		bridge.add_subscriber(joint_state_sub);

	}

	bridge.run();
}
