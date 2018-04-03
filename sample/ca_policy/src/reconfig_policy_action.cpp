
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
#include <fstream>
#include <ros/ros.h>
#include "ca_policy/reconfig_policy_action.h"

namespace intelligent_ca
{
ReconfigPolicyAction::ReconfigPolicyAction()
{
}

ReconfigPolicyAction::~ReconfigPolicyAction()
{
}

void ReconfigPolicyAction::execute()
{
  if (config_file_.length() == 0)
  {
    ROS_WARN("\tConfig file path is not set, do nothing!");
    return;
  }

  ROS_INFO("\texecuting ReconfigPolicyAction [%s]", config_file_.c_str());
  std::string cmd = "rosrun dynamic_reconfigure dynparam load /move_base/DWAPlannerROS ";
  cmd += config_file_;
  cmd += "&";
  system(cmd.c_str());
}

bool ReconfigPolicyAction::setConfiguration(const std::string& config)
{
  ROS_INFO("\tsetting Config file for ReconfigPolicyAction [%s]...", config.c_str());

  std::ifstream ifile(config.c_str());
  if (ifile)
  {
    config_file_ = config;
    return true;
  }
  else
  {
    ROS_ERROR("\tConfig file path is not correct. Please check param [policy_base_dir] and [CaPoicies/config] settings.");
    config_file_ = "";
    return false;
  }
}

}  // namespace
