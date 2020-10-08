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

#include "joint_state_subscriber.h"
#include "utils.h"
#include "logger.h"
#include "profiles.h"
#include "sdo_error.h"
#include "ros/ros.h"
#include <string>

namespace kaco {

  JointStateSubscriber::JointStateSubscriber(Device & device, int32_t position_0_degree,
      int32_t position_360_degree, std::string topic_name): m_device(device), m_position_0_degree(position_0_degree),
      m_position_360_degree(position_360_degree), m_topic_name(topic_name),
      m_initialized(false) {

      const uint16_t profile = device.get_device_profile_number();

      if (profile != 402) {
        throw std::runtime_error("JointStatePublisher can only be used with a CiA 402 device."
          " You passed a device with profile number " + std::to_string(profile));
      }

      const Value operation_mode = device.get_entry("Modes of operation display");

      // TODO: look into INTERPOLATED_POSITION_MODE
      //	if (operation_mode != Profiles::constants.at(402).at("profile_position_mode")
      //		&& operation_mode != Profiles::constants.at(402).at("interpolated_position_mode")) {
      //		throw std::runtime_error("[JointStatePublisher] Only position mode supported yet."
      //			" Try device.set_entry(\"modes_of_operation\", device.get_constant(\"profile_position_mode\"));");
      //	}

      if (operation_mode == Profiles::constants.at(402).at("profile_position_mode")) {
        operation_mode_ = PROFILE_POSITION;
        PRINT("Subscriber for profile_position_mode");
      } else if (operation_mode == Profiles::constants.at(402).at("profile_velocity_mode")) {
        operation_mode_ = PROFILE_VELOCITY;
        PRINT("Subscriber for profile_velocity_mode");
      } else if (operation_mode == Profiles::constants.at(402).at("current_mode")) {
        operation_mode_ = CURRENT_MODE;
        PRINT("Subscriber for current_mode");
      } else {

        throw std::runtime_error("[JointStatePublisher] Only position mode supported yet."
          " Try device.set_entry(\"modes_of_operation\", device.get_constant(\"profile_position_mode\"));");
      }

      if (m_topic_name.empty()) {
        uint8_t node_id = device.get_node_id();
        m_topic_name = "device" + std::to_string(node_id) + "/set_joint_state";
      }

      motor_id = device.get_node_id();

    }

  void JointStateSubscriber::advertise() {

    assert(!m_topic_name.empty());
    ROS_DEBUG_STREAM("Advertising " << m_topic_name);
    ros::NodeHandle nh;
    m_subscriber = nh.subscribe(m_topic_name, queue_size, & JointStateSubscriber::receive, this, ros::TransportHints().tcpNoDelay());
    //m_subscriber = nh.subscribe(m_topic_name, queue_size, & JointStateSubscriber::receive, this);
    m_initialized = true;
    m_subscribe_state = true;
  }

  void JointStateSubscriber::receive(const sensor_msgs::JointState & msg) {

    try {
        if (!m_subscribe_state) {
            WARN("[EntryPublisher] m_subscribe_state is not 'true', not subscribing anything (tip: call set_subscribe_state(true);)");
            return;
        }

      if (operation_mode_ == PROFILE_POSITION) {
      	if(msg.position.size() <= 0){
      		ROS_WARN("subscriber %s received EMPTY JointState position", m_topic_name.c_str());
      		return;
      	}

        //ROS_INFO_STREAM("Received JointState message [Position] " << msg.position[0]);
        m_device.execute("set_target_position", static_cast < int32_t > (msg.position[0]));
      } 

      else if (operation_mode_ == PROFILE_VELOCITY) {
      	if(msg.velocity.size() <= 0){
      		ROS_WARN("subscriber %s received EMPTY JointState velocity", m_topic_name.c_str());
      		return;
      	}

	        //ROS_INFO_STREAM("Received JointState message [Velocity] " << msg.velocity[0]);
	    m_device.execute("set_target_velocity", static_cast < int32_t > (msg.velocity[0]));
      }

      else if (operation_mode_ == CURRENT_MODE) {
      	if(msg.effort.size() <= 0){
      		ROS_WARN("subscriber %s received EMPTY JointState effort", m_topic_name.c_str());
      		return;
      	}

      	double current_bias = 0.0;
      	if(motor_id == 1){
      		current_bias = 130.0;
      	}
      	else if(motor_id == 2){
      		current_bias = -83.0;
      	}
      	else if(motor_id == 3){
      		current_bias = 86.0;
      	}
      	else if(motor_id == 4){
      		current_bias = 40.0;
      	}
      	else if(motor_id == 5){
      		current_bias = 300.0;
      	}
      	else if(motor_id == 6){
      		current_bias = 70.0;
      	}
      	else{
      		current_bias = 0.0;
      	}

  		m_device.execute("set_target_current", static_cast < int16_t > (torque_to_current(msg.effort[0]) + current_bias));
    	// m_device.execute("set_target_current", static_cast < int16_t > (msg.effort[0]));
      }

    } catch (const sdo_error & error) {
      // TODO: only catch timeouts?
      ERROR("Exception in JointStateSubscriber::receive(): " << error.what());
    }
  }

  int32_t JointStateSubscriber::rad_to_pos(double rad) const {

    const double p = rad;
    const double min = m_position_0_degree;
    const double max = m_position_360_degree;
    const double dist = max - min;
    const double result = ((p / (2 * pi())) * dist) + min;
    return (int32_t) result;

  }


  int16_t JointStateSubscriber::torque_to_current(double torque) const {
  	// 0.0259 Nm/A ---> mA = T/0.0000259

  	const double ratio = 0.0000259;
    double result = torque/ratio;
    if (result > 2700){
    	result = 2700.0;
    }
    else if (result < -2700){
    	result = -2700.0;
    }
    return (int16_t) result;

  }

  void JointStateSubscriber::set_subscribe_state(bool state) {
      m_subscribe_state = state;
  }

} // end namespace kaco
