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
 
#include "test_entry_publisher.h"
#include "utils.h"
#include "logger.h"
#include "rclcpp/rclcpp.hpp"
#include "sdo_error.h"

#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include <string>

namespace kaco {

TestEntryPublisher::TestEntryPublisher(uint8_t device, const std::string& entry_name, const ReadAccessMethod access_method)
	: m_device(device), m_entry_name(entry_name), m_access_method(access_method)
{

	uint8_t node_id = device;
	m_device_prefix = "device" + std::to_string(node_id) + "/";
	// no spaces and '-' allowed in ros names
	m_name = Utils::escape(entry_name);
}

void TestEntryPublisher::advertise() {

	std::string topic = m_device_prefix+"get_"+m_name;
	DEBUG_LOG("Advertising "<<topic);
	
	if (!m_node) {
		ERROR("[TestEntryPublisher] Node not set. Call set_node() first.");
		return;
	}
	
	// For test purposes, create a UInt8 publisher
	m_publisher = m_node->create_publisher<std_msgs::msg::UInt8>(topic, queue_size);
	m_publish_state = true;
}

void TestEntryPublisher::set_publish_state(bool state) {
	m_publish_state = state;
}

void TestEntryPublisher::publish() {

	if (!m_publish_state) {
		RCLCPP_WARN(m_node->get_logger(), "[TestEntryPublisher] m_publish_state is not 'true', not publishing anything (tip: call set_publish_state(true);)");
		return;
	}

	try {

		// Publish test data
		std_msgs::msg::UInt8 msg;
		msg.data = 123; // Test value
		auto typed_pub = std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::UInt8>>(m_publisher);
		typed_pub->publish(msg);
		
	} catch (const sdo_error& error) {
		// TODO: only catch timeouts?
		ERROR("Exception in TestEntryPublisher::publish(): "<<error.what());
	}

}

} // end namespace kaco