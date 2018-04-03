
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
#include <ros/ros.h>
#include "ca_policy/ca_policy.h"
#include "moving_object_msgs/MovingObject.h"
#include <tf/transform_listener.h>

namespace intelligent_ca
{
CaPolicy::CaPolicy(const std::string name) : name_(name)
{
}

CaPolicy::~CaPolicy()
{
}

void CaPolicy::addAction(const std::shared_ptr<PolicyAction> action)
{
  actions_.push_back(action);
}

void CaPolicy::execute()
{
  for (auto action : actions_)
  {
    action->execute();
  }
}

}  // namespace
