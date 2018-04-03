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
#include <vector>
#include <ros/ros.h>
#include "ca_policy/ca_policy.h"
#include "ca_policy/ca_policy_pool.h"
#include "moving_object_msgs/MovingObject.h"

namespace intelligent_ca
{
CaPolicyPool::CaPolicyPool()
{
  policies_.clear();
}

CaPolicyPool::~CaPolicyPool()
{
  policies_.clear();
}

bool CaPolicyPool::addPolicy(const std::string name, const std::shared_ptr<CaPolicy>& policy)
{
  try
  {
    CaPolicyVector::iterator exist = findPolicy(name);
    if (exist != policies_.end())
    {
      ROS_INFO("Policy %s already exist, update it with new content.", name.c_str());
      std::get<1>(*exist) = policy;
      return true;
    }

    ROS_INFO("Create new policy %s ...", name.c_str());
    CaPolicyPair policy_pair = std::make_pair(name, policy);
    policies_.push_back(policy_pair);
    return true;
  }
  catch (std::bad_alloc& ba)
  {
    ROS_WARN("bad_alloc caught: %s", ba.what());
    return false;
  }
  catch (...)
  {
    ROS_WARN("Error when add policy...");
    return false;
  }
}

bool CaPolicyPool::deletePolicy(const std::string name)
{
  CaPolicyVector::iterator exist = findPolicy(name);
  if (exist != policies_.end())
  {
    policies_.erase(exist);
    return true;
  }

  return false;
}

bool CaPolicyPool::isPolicyExist(const std::string name)
{
  for (CaPolicyVector::iterator it = policies_.begin(); it != policies_.end(); ++it)
  {
    if (name == std::get<0>(*it))
    {
      return true;
    }
  }

  return false;
}

CaPolicyVector::iterator CaPolicyPool::findPolicy(const std::string name)
{
  CaPolicyVector::iterator it = policies_.begin();
  for (; it != policies_.end(); ++it)
  {
    if (name == std::get<0>(*it))
    {
      break;
    }
  }

  return it;
}

std::string CaPolicyPool::getCurrentPolicy()
{
  return std::get<0>(current_policy_);
}
bool CaPolicyPool::setCurrentPolicy(const std::string name)
{
  CaPolicyVector::iterator exist = findPolicy(name);
  if (exist != policies_.end())
  {
    current_policy_ = *exist;
    ROS_INFO("Enabling CA Policy [%s] ...", name.c_str());
    std::get<1>(current_policy_)->execute();
    return true;
  }
  else
  {
    ROS_WARN("No policy named %s", name.c_str());
    return false;
  }
}

}  // namespace
