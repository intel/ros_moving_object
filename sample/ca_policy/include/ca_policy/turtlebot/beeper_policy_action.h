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
#ifndef ICA_TURTLEBOT_BEEPER_POLICY_ACTION_H
#define ICA_TURTLEBOT_BEEPER_POLICY_ACTION_H

#include <ros/ros.h>
#include <kobuki_msgs/Sound.h>
#include "ca_policy/policy_action.h"

namespace intelligent_ca
{
namespace turtlebot2
{

/** @brief Policy Actions for LED Belt of WaterBot.
 *  Note that maybe it doesn't match others robot base.
 */
class BeeperPolicyAction : public PolicyAction
{
public:
  BeeperPolicyAction();
  virtual ~BeeperPolicyAction();

  /** @brief execute the policy*/
  void execute();

  /** @brief set state to LED State..
   *  @param[in] state The LED state to be set.
   *  @return true if set successfully.
   */
  bool setBeeperState(const unsigned char state)
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
  unsigned char getBeeperState()
  {
    return state_;
  }

protected:
  ros::NodeHandle private_nh_;

  /**< Beeper State to be set. */
  unsigned char state_;

  ros::Publisher ca_beeper_pub_;
};

}  // namespace turtlebot2
}  // namespace intelligent_ca
#endif
