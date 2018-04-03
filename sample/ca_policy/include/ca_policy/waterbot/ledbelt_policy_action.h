/*
 * Copyright (c) 2017~2018 Intel Corporation
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
#ifndef ICA_LEDBELT_POLICY_ACTION_H
#define ICA_LEDBELT_POLICY_ACTION_H

#include <ros/ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt8MultiArray.h>
#include "ca_policy/policy_action.h"

namespace intelligent_ca
{
namespace waterbot
{
/**< LED States for Water Robot Base. */
enum LEDState
{
  CA_LED_OFF = 0,
  CA_LED_ON,
  CA_LED_BLINK,
  CA_LED_BREATHE
};
/** @brief Policy Actions for LED Belt of WaterBot.
 *  Note that maybe it doesn't match others robot base.
 */
class LedBeltPolicyAction : public PolicyAction
{
public:
  LedBeltPolicyAction();
  virtual ~LedBeltPolicyAction();

  /** @brief execute the policy*/
  void execute();

  /** @brief set state to LED State..
   *  @param[in] state The LED state to be set.
   *  @return true if set successfully.
   */
  bool setLedState(const LEDState state)
  {
    state_ = state;
    return true;
  }

  /** @brief set state to LED State..
   *  @param[in] state The LED state to be set.
   *  @return true if set successfully.
   */
  bool setLuminance(const int lum);

  /** @brief  Get the current LED State .
   *  @return LED State archived in class member variable state_.
   */
  LEDState getLedState()
  {
    return state_;
  }

protected:
  ros::NodeHandle private_nh_;
  std_msgs::UInt8MultiArray led_array_;

  int luminance_;
  /** Format of led_array_.data:
   *  data[0]: LED state, see enum LEDState.
   *  data[1]: 0, not used.
   *  data[2]: 0, not used.
   *  data[3]: luminance [0, 255]
   */

  /**< LED State to be set. */
  LEDState state_;

  ros::Publisher ca_led_pub_;
};

}  // namespace intelligent_ca
}  // namespace waterbot
#endif
