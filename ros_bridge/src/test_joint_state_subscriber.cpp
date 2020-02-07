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

#include "test_joint_state_subscriber.h"
#include "utils.h"
#include "logger.h"
#include "profiles.h"
#include "sdo_error.h"
#include "ros/ros.h"
#include <string>
#include <thread>
#include <chrono>

namespace kaco {

  TestJointStateSubscriber::TestJointStateSubscriber(uint8_t device, int32_t position_0_degree,
      int32_t position_360_degree, std::string topic_name): m_device(device), m_position_0_degree(position_0_degree),
    m_position_360_degree(position_360_degree), m_topic_name(topic_name),
    m_initialized(false) {

      
        operation_mode_ = PROFILE_VELOCITY;

      if (m_topic_name.empty()) {
        uint8_t node_id = device;
        m_topic_name = "device" + std::to_string(node_id) + "/set_joint_state";
      }

    }

  void TestJointStateSubscriber::advertise() {

    assert(!m_topic_name.empty());
    ROS_DEBUG_STREAM("Advertising " << m_topic_name);
    ros::NodeHandle nh;
    m_subscriber = nh.subscribe(m_topic_name, queue_size, & TestJointStateSubscriber::receive, this, ros::TransportHints().tcpNoDelay());
    //m_subscriber = nh.subscribe(m_topic_name, queue_size, & JointStateSubscriber::receive, this);
    m_initialized = true;
    m_subscribe_state = true;

    // Test callback - DELETEME
    //m_subscriber1 = nh.subscribe(m_topic_name, queue_size, & JointStateSubscriber::receiveTest, this, ros::TransportHints().tcpNoDelay());

  }

  void TestJointStateSubscriber::receive(const sensor_msgs::JointState & msg) {

      if (!m_subscribe_state) {
          WARN("[EntryPublisher] m_subscribe_state is not 'true', not subscribing anything (tip: call set_subscribe_state(true);)");
          return;
      }
    ROS_INFO("JointStateSubscriber receive -- called %s", m_topic_name.c_str());
    // ROS_INFO("SLEEPING...");
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // ROS_INFO("AWAKE...");
  }

  int32_t TestJointStateSubscriber::rad_to_pos(double rad) const {

    const double p = rad;
    const double min = m_position_0_degree;
    const double max = m_position_360_degree;
    const double dist = max - min;
    const double result = ((p / (2 * pi())) * dist) + min;
    return (int32_t) result;

  }

  void TestJointStateSubscriber::set_subscribe_state(bool state) {
      m_subscribe_state = state;
  }

} // end namespace kaco