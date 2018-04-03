
/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ca_policy/turtlebot/led_policy_action.h"
#include <kobuki_msgs/Led.h>

namespace intelligent_ca
{
namespace turtlebot2
{
LedPolicyAction::LedPolicyAction() : private_nh_{ "~" }
{
  ca_led_pub_ = private_nh_.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 1);
}

LedPolicyAction::~LedPolicyAction()
{
}

void LedPolicyAction::execute()
{
  ROS_INFO("\texecuting turtlebot Led Policy Action...");
  kobuki_msgs::Led led;
  led.value = state_;

  ca_led_pub_.publish(led);
}

}  // namespace waterbot
}  // namespace intelligent_ca
