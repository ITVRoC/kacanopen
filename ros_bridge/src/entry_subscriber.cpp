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
 
#include "entry_subscriber.h"
#include "utils.h"
#include "logger.h"
#include "rclcpp/rclcpp.hpp"
#include "sdo_error.h"

#include <string>
#include <functional>

namespace kaco {

EntrySubscriber::EntrySubscriber(Device& device, const std::string& entry_name, const WriteAccessMethod access_method)
	: m_device(device), m_entry_name(entry_name), m_access_method(access_method)
{

	uint8_t node_id = device.get_node_id();
	m_device_prefix = "device" + std::to_string(node_id) + "/";
	// no spaces and '-' allowed in ros names
	m_name = Utils::escape(entry_name);
	m_type = device.get_entry_type(entry_name);

}

void EntrySubscriber::advertise() {
	
	std::string topic = m_device_prefix+"set_"+m_name;
	DEBUG_LOG("Advertising "<<topic);
	if (!m_node) {
		ERROR("[EntrySubscriber] Node not set. Call set_node() first.");
		return;
	}

	switch(m_type) {
		case Type::uint8:
			m_subscriber = m_node->create_subscription<std_msgs::msg::UInt8>(
				topic, queue_size, std::bind(&EntrySubscriber::receive_uint8, this, std::placeholders::_1));
			break;
		case Type::uint16:
			m_subscriber = m_node->create_subscription<std_msgs::msg::UInt16>(
				topic, queue_size, std::bind(&EntrySubscriber::receive_uint16, this, std::placeholders::_1));
			break;
		case Type::uint32:
			m_subscriber = m_node->create_subscription<std_msgs::msg::UInt32>(
				topic, queue_size, std::bind(&EntrySubscriber::receive_uint32, this, std::placeholders::_1));
			break;
		case Type::int8:
			m_subscriber = m_node->create_subscription<std_msgs::msg::Int8>(
				topic, queue_size, std::bind(&EntrySubscriber::receive_int8, this, std::placeholders::_1));
			break;
		case Type::int16:
			m_subscriber = m_node->create_subscription<std_msgs::msg::Int16>(
				topic, queue_size, std::bind(&EntrySubscriber::receive_int16, this, std::placeholders::_1));
			break;
		case Type::int32:
			m_subscriber = m_node->create_subscription<std_msgs::msg::Int32>(
				topic, queue_size, std::bind(&EntrySubscriber::receive_int32, this, std::placeholders::_1));
			break;
		case Type::boolean:
			m_subscriber = m_node->create_subscription<std_msgs::msg::Bool>(
				topic, queue_size, std::bind(&EntrySubscriber::receive_boolean, this, std::placeholders::_1));
			break;
		case Type::string:
			m_subscriber = m_node->create_subscription<std_msgs::msg::String>(
				topic, queue_size, std::bind(&EntrySubscriber::receive_string, this, std::placeholders::_1));
			break;
		default:
			ERROR("[EntryPublisher::advertise] Invalid entry type.")
	}
    m_subscribe_state = true;
}

void EntrySubscriber::receive_uint8(const std_msgs::msg::UInt8& msg) {

    if (!m_subscribe_state) {
        WARN("[EntryPublisher] m_subscribe_state is not 'true', not subscribing anything (tip: call set_subscribe_state(true);)");
        return;
    }

	try {
		DEBUG_LOG("Recieved msg: "<<msg.data);
		m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
	} catch (const sdo_error& error) {
		// TODO: only catch timeouts?
		ERROR("Exception in EntrySubscriber::receive_uint8(): "<<error.what());
	}
}

void EntrySubscriber::receive_uint16(const std_msgs::msg::UInt16& msg) {

    if (!m_subscribe_state) {
        WARN("[EntryPublisher] m_subscribe_state is not 'true', not subscribing anything (tip: call set_subscribe_state(true);)");
        return;
    }

	try {
		DEBUG_LOG("Recieved msg: "<<msg.data);
		m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
	} catch (const sdo_error& error) {
		// TODO: only catch timeouts?
		ERROR("Exception in EntrySubscriber::receive_uint16(): "<<error.what());
	}
}

void EntrySubscriber::receive_uint32(const std_msgs::msg::UInt32& msg) {

    if (!m_subscribe_state) {
        WARN("[EntryPublisher] m_subscribe_state is not 'true', not subscribing anything (tip: call set_subscribe_state(true);)");
        return;
    }

	try {
		DEBUG_LOG("Recieved msg: "<<msg.data);
		m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
	} catch (const sdo_error& error) {
		// TODO: only catch timeouts?
		ERROR("Exception in EntrySubscriber::receive_uint32(): "<<error.what());
	}
}

void EntrySubscriber::receive_int8(const std_msgs::msg::Int8& msg) {

    if (!m_subscribe_state) {
        WARN("[EntryPublisher] m_subscribe_state is not 'true', not subscribing anything (tip: call set_subscribe_state(true);)");
        return;
    }

	try {
		DEBUG_LOG("Recieved msg: "<<msg.data);
		m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
	} catch (const sdo_error& error) {
		// TODO: only catch timeouts?
		ERROR("Exception in EntrySubscriber::receive_int8(): "<<error.what());
	}
}

void EntrySubscriber::receive_int16(const std_msgs::msg::Int16& msg) {

    if (!m_subscribe_state) {
        WARN("[EntryPublisher] m_subscribe_state is not 'true', not subscribing anything (tip: call set_subscribe_state(true);)");
        return;
    }

	try {
		DEBUG_LOG("Recieved msg: "<<msg.data);
		m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
	} catch (const sdo_error& error) {
		// TODO: only catch timeouts?
		ERROR("Exception in EntrySubscriber::receive_int16(): "<<error.what());
	}
}

void EntrySubscriber::receive_int32(const std_msgs::msg::Int32& msg) {

    if (!m_subscribe_state) {
        WARN("[EntryPublisher] m_subscribe_state is not 'true', not subscribing anything (tip: call set_subscribe_state(true);)");
        return;
    }

	try {
		DEBUG_LOG("Recieved msg: "<<msg.data);
		m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
	} catch (const sdo_error& error) {
		// TODO: only catch timeouts?
		ERROR("Exception in EntrySubscriber::receive_int32(): "<<error.what());
	}
}

void EntrySubscriber::receive_boolean(const std_msgs::msg::Bool& msg) {

    if (!m_subscribe_state) {
        WARN("[EntryPublisher] m_subscribe_state is not 'true', not subscribing anything (tip: call set_subscribe_state(true);)");
        return;
    }

	try {
		DEBUG_LOG("Recieved msg: "<<msg.data);
		m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
	} catch (const sdo_error& error) {
		// TODO: only catch timeouts?
		ERROR("Exception in EntrySubscriber::receive_boolean(): "<<error.what());
	}
}

void EntrySubscriber::receive_string(const std_msgs::msg::String& msg) {

    if (!m_subscribe_state) {
        WARN("[EntryPublisher] m_subscribe_state is not 'true', not subscribing anything (tip: call set_subscribe_state(true);)");
        return;
    }

	try {
		DEBUG_LOG("Recieved msg: "<<msg.data);
		m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
	} catch (const sdo_error& error) {
		// TODO: only catch timeouts?
		ERROR("Exception in EntrySubscriber::receive_string(): "<<error.what());
	}
}


void EntrySubscriber::set_subscribe_state(bool state) {
    m_subscribe_state = state;
}

} // end namespace kaco