
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

#include "ca_policy/waterbot/ledbelt_policy_action.h"

namespace intelligent_ca
{
namespace waterbot
{
LedBeltPolicyAction::LedBeltPolicyAction() : private_nh_{ "~" }
{
  ca_led_pub_ = private_nh_.advertise<std_msgs::UInt8MultiArray>("/water_uavcan_master/set_led", 1);
}

LedBeltPolicyAction::~LedBeltPolicyAction()
{
}

void LedBeltPolicyAction::execute()
{
  ROS_INFO("\texecuting waterbot Led Belt Policy Action...");
  led_array_.data.clear();
  led_array_.data.resize(4, 0);

  /**< set LED state. */
  led_array_.data[0] = state_;

  /**< set LED luminance. */
  led_array_.data[3] = luminance_;

  ca_led_pub_.publish(led_array_);
}

bool LedBeltPolicyAction::setLuminance(const int lum)
{
  ROS_INFO("\tSetting LED Luminance [%d]...", lum);

  luminance_ = lum;

  if (luminance_ < 0)
  {
    luminance_ = 0;
  }
  if (luminance_ > 255)
  {
    luminance_ = 255;
  }

  return true;
}

}  // namespace waterbot
}  // namespace intelligent_ca
