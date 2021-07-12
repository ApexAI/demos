// Copyright 2021 Apex.AI, Inc.
// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "example_interfaces/msg/shm_topic.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp {
class ShmDemoListener : public rclcpp::Node {
private:
  using Topic = example_interfaces::msg::ShmTopic;

public:
  DEMO_NODES_CPP_PUBLIC
  explicit ShmDemoListener(const rclcpp::NodeOptions &options)
      : Node("shm_demo_listener", options) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // subscription callback to process arriving data
    auto callback = [this](const Topic::SharedPtr msg) -> void {
      // Read the message and perform operations accordingly.

      // transform to string for output
      // Note: dealing with std::array as underlying type fixed size string is
      // not nice to use.

      std::memcpy(m_buffer, msg->data.data(), msg->size);
      m_buffer[255] = '\0'; // in case there was no proper zero termination

      RCLCPP_INFO(this->get_logger(), "Received: %s %lu", m_buffer,
                  msg->counter);
    };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    m_subscription = create_subscription<Topic>("chatter", qos, callback);
  }

private:
  rclcpp::Subscription<Topic>::SharedPtr m_subscription;

  char m_buffer[256];
};

} // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ShmDemoListener)
