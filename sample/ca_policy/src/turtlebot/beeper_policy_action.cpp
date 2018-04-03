
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

#include "ca_policy/turtlebot/beeper_policy_action.h"

namespace intelligent_ca
{
namespace turtlebot2
{
BeeperPolicyAction::BeeperPolicyAction() : private_nh_{ "~" }
{
  ca_beeper_pub_ = private_nh_.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);
}

BeeperPolicyAction::~BeeperPolicyAction()
{
}

void BeeperPolicyAction::execute()
{
  ROS_INFO("\texecuting turtlebot Sound Policy Action...");
  kobuki_msgs::Sound beep;
  beep.value = state_;

  ca_beeper_pub_.publish(beep);
}

}  // namespace turtlebot2
}  // namespace intelligent_ca
