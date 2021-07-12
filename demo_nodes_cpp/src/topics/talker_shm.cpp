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

#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include "demo_nodes_cpp/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcutils/cmdline_parser.h"

#include "example_interfaces/msg/shm_topic.hpp"

using namespace std::chrono_literals;

namespace demo_nodes_cpp {

class ShmDemoTalker : public rclcpp::Node {
private:
  using Topic = example_interfaces::msg::ShmTopic;

public:
  DEMO_NODES_CPP_PUBLIC
  explicit ShmDemoTalker(const rclcpp::NodeOptions &options)
      : Node("shm_demo_talker", options) {

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    auto publishMessage = [this]() -> void {
      auto loanedMsg = m_publisher->borrow_loaned_message();

      populateLoanedMessage(loanedMsg);

      m_publisher->publish(std::move(loanedMsg));

      m_count++;
    };

    rclcpp::QoS qos(rclcpp::KeepLast(7));
    m_publisher = this->create_publisher<Topic>("my_shm_topic", qos);

    // Use a timer to schedule periodic message publishing.
    m_timer = this->create_wall_timer(1s, publishMessage);
  }

private:
  uint64_t m_count = 1;
  rclcpp::Publisher<Topic>::SharedPtr m_publisher;
  rclcpp::TimerBase::SharedPtr m_timer;

  void populateLoanedMessage(rclcpp::LoanedMessage<Topic> &loanedMsg) {
    Topic &msg = loanedMsg.get();

    std::string hello{"Hello World"};

    // not a nice way to fill the data but msg.data is a std::array
    // we should also assume that in practice the message string is not constant
    // this actually circumvents the C++ type system
    std::strncpy((char *)msg.data.data(), hello.data(), 256);

    // This copy here should be seen as creating/generating the data in place.
    // Depending on whether the loaned message initializes the memory of msg
    // this is not completely possible as of now.
    // I.e. it is default initialized and then overwritten here, incurring
    // unnecessary overhead.

    msg.counter = m_count;

    RCLCPP_INFO(this->get_logger(), "Publishing: %s %lu", hello.c_str(),
                msg.counter);
  }
};

} // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::ShmDemoTalker)