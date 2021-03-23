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

#ifndef SUBSCRIBE_TEST__SUBSCRIBE_TEST_CORE_HPP_
#define SUBSCRIBE_TEST__SUBSCRIBE_TEST_CORE_HPP_

#include <memory>
#include <mutex>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

class SubscribeTest : public rclcpp::Node
{
public:
  SubscribeTest();

private:
  // subscriber
  rclcpp::Subscription<autoware_planning_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<autoware_planning_msgs::msg::PathWithLaneId>::SharedPtr
    sub_path_with_lane_id_;

  // variable
  std::mutex mutex_;
  autoware_planning_msgs::msg::Path path_;
  int count = 0 ;
  

  // callback function
  void callbackAutowarePath(const autoware_planning_msgs::msg::Path::ConstSharedPtr msg_ptr);

  void callbackAutowarePathWithLaneId(
    const autoware_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_ptr);
};

#endif  // SUBSCRIBE_TEST__SUBSCRIBE_TEST_CORE_HPP_
