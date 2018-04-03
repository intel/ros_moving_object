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

#ifndef ICA_CA_POLICY_POOL_H
#define ICA_CA_POLICY_POOL_H

#include <ros/ros.h>
#include <vector>

#include "ca_policy.h"
#include <ca_policy/ca_policy.h>

namespace intelligent_ca
{
using CaPolicyPair = std::pair<std::string, std::shared_ptr<CaPolicy>>;
using CaPolicyVector = std::vector<CaPolicyPair>;

const std::string kCaPolicy_Normal = "normal";
const std::string kCaPolicy_Social = "social";

/** @brief This class manages the CA policies in use.
 * add, delete, set policy name, set current policy.
 */
class CaPolicyPool
{
public:
  CaPolicyPool();
  virtual ~CaPolicyPool();

  /** @brief add CA policy by the given name and policy instance.
   *  @param[in] name   The name of the policy to be added.
   *  @param[in] policy Policy to be added.
   *  @return true if successfully added, otherwise false.
   */
  bool addPolicy(const std::string name, const std::shared_ptr<CaPolicy>& policy);

  /** @brief delete CA policy by the give policy name.
   *  @param[in] name The name of the policy to be deleted.
   *  @return true if successfully deleted, otherwise false.
   */
  bool deletePolicy(const std::string name);

  /** @brief Get the name of the current CA policy.
   *  @return the name of the current CA policy.
   */
  std::string getCurrentPolicy();

  /** @brief Enable a given CA policy.
   *  @param[in] name The name of the CA policy to be enabled.
   *  @return true if successfully set. false otherwise.
   */
  bool setCurrentPolicy(const std::string name);

  /** @brief Check a given policy exists or not.
   *  @param[in] name The name of the CA policy to be checked.
   *  @return true if policy _name_ is existed. false otherwise.
   */
  bool isPolicyExist(const std::string name);

private:
  /** @brief Search and return the CA policy by the given name.
   *  @param[in] name The name of the policy to be searched.
   *  @param[out] pair The iterator of the policy found.
   *  @return The iterator of the policy if found, otherwise iterator::end().
   */
  CaPolicyVector::iterator findPolicy(const std::string name);

  CaPolicyVector policies_;     /**< @brief CA policy list. */
  CaPolicyPair current_policy_; /**< @brief The working CA policy, which is used to configure navigation. */
};

}  // namespace
#endif
