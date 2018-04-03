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
#ifndef ICA_RECONFIG_POLICY_ACTION_H
#define ICA_RECONFIG_POLICY_ACTION_H

#include "ca_policy/policy_action.h"

namespace intelligent_ca
{
/** @brief CA Policy description.
 * This class stores any information about CA Policy.
 * each policy holds a name and a config file.
 */
class ReconfigPolicyAction : public PolicyAction
{
public:
  ReconfigPolicyAction();
  virtual ~ReconfigPolicyAction();

  /** @brief execute the policy*/
  void execute();

  /** @brief set policy's configuration file.
   *  @param[in] config the config file path to be set to the Ca Policy.
   *  @return    true if successfully set, otherwise false.
   */
  bool setConfiguration(const std::string& config);

  /** @brief  Get the config file path for the ca policy.
   *  @return The string storing the policy's config file path.
   */
  std::string getConfiguration()
  {
    return config_file_;
  }

protected:
  std::string config_file_; /**< @brief configuration file path. */
};

}  // namespace
#endif
