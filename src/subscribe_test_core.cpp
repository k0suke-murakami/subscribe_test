// Copyright 2020 Tier IV, Inc.
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
#include <functional>
#include <memory>
#include <utility>

#include "subscribe_test/subscribe_test_core.hpp"

using std::placeholders::_1;

SubscribeTest::SubscribeTest()
: Node("subscribe_test", rclcpp::NodeOptions().allow_undeclared_parameters(true))
{
  // subscriber

  rclcpp::QoS qos{1};
  sub_path_ = this->create_subscription<autoware_planning_msgs::msg::Path>(
    "input/autoware_path", 1, std::bind(&SubscribeTest::callbackAutowarePath, this, _1));
  sub_path_with_lane_id_ = this->create_subscription<autoware_planning_msgs::msg::PathWithLaneId>(
    "input/autoware_path_with_lane_id", 1,
    std::bind(&SubscribeTest::callbackAutowarePathWithLaneId, this, _1));
}

void SubscribeTest::callbackAutowarePath(
  const autoware_planning_msgs::msg::Path::ConstSharedPtr msg_ptr)
{
  auto t_start = std::chrono::high_resolution_clock::now();
  const autoware_planning_msgs::msg::Path path = *msg_ptr;
  const int iter = count % 10 == 0 ? 1000000 : 10000;
  count++;
  for (int i = 0; i < iter; i++) {
    int * test;
    const int as = 100000;  // array size
    test = (int *)malloc(sizeof(int) * as);
    test[0] = (int)msg_ptr->header.stamp.sec;
    test[1] = (int)msg_ptr->header.stamp.sec;
    const auto t = test[0] * test[1];
    RCLCPP_DEBUG_STREAM(this->get_logger(), "t " << t);
    free(test);
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  float elapsed_ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
  RCLCPP_INFO_STREAM(this->get_logger(), "Path Subscribe " << elapsed_ms << "ms");
}

void SubscribeTest::callbackAutowarePathWithLaneId(
  const autoware_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_ptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "PathWithLI-Subscribe");
}
