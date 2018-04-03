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
#include <vector>
#include <fstream>
#include "ca_policy/ca_policy_ros.h"
#include "ca_policy/ca_policy_pool.h"
#include "ca_policy/waterbot/ledbelt_policy_action.h"
#include "ca_policy/turtlebot/led_policy_action.h"
#include "ca_policy/turtlebot/beeper_policy_action.h"
#include "ca_policy/reconfig_policy_action.h"
#include <ca_policy_msgs/CaPolicy.h>

namespace intelligent_ca
{
const std::string kRobotBase_WaterBot = "waterbot";
const std::string kRobotBase_TurtleBot = "turtlebot";
const std::string kTopicCaPolicy = "ca_policy";
//const std::string kTopicSocialObjectInFrame = "/moving_object/social_object";
const std::string kTopicSocialObjectInFrame = "social_object";

CaPolicyRos::CaPolicyRos(ros::NodeHandle& nh) :
    node_handler_(nh), last_set_time_(0)
{
  ROS_INFO("ENTER CaPolicyRos Constructor...");
  init();
}

CaPolicyRos::~CaPolicyRos()
{
}

void CaPolicyRos::init()
{
  std::string config_path;

  node_handler_.param("robot_base", robot_base_name_, kRobotBase_WaterBot);

  if (config_path.length() > 0 && config_path[config_path.length() - 1] != '/')
  {
    config_path += "/";
  }

  if (node_handler_.hasParam("CaPolicies"))
  {
    XmlRpc::XmlRpcValue my_list;
    node_handler_.getParam("CaPolicies", my_list);
    for (int32_t i = 0; i < my_list.size(); ++i)
    {
      std::string pname = static_cast<std::string>(my_list[i]["name"]);
      std::string config = static_cast<std::string>(my_list[i]["config"]);
      ROS_INFO("Using CA Policy \"%s:%s\"", pname.c_str(), config.c_str());

      // check config path
      std::ifstream ifile(config.c_str());
      if (ifile)
      {
        config_path = config;
      }
      else
      {
        node_handler_.param("policy_base_dir", config_path, std::string(""));
        if (config_path.length() > 0 && config_path[config_path.length() - 1] != '/')
        {
          config_path += "/";
        }
        config_path += config;
      }

      /**< Add Dynamic Reconfig Action. */
      std::shared_ptr<CaPolicy> policy = std::make_shared < CaPolicy > (pname);
      std::shared_ptr<ReconfigPolicyAction> reconfig_action = std::make_shared<ReconfigPolicyAction>();
      reconfig_action->setConfiguration(config_path);
      policy->addAction(reconfig_action);

      /**< for waterbot */
      if (robot_base_name_ == kRobotBase_WaterBot)
      {
        /**< Add LED Belt Action*/
        std::shared_ptr<waterbot::LedBeltPolicyAction> led_action = std::make_shared<waterbot::LedBeltPolicyAction>();
        if (pname == kCaPolicy_Social)
        {
          led_action->setLedState(intelligent_ca::waterbot::CA_LED_BLINK);
        }
        else
        {
          led_action->setLedState(intelligent_ca::waterbot::CA_LED_ON);
        }
        int lumin;
        node_handler_.param("luminance", lumin, 128);
        led_action->setLuminance(lumin);

        policy->addAction(led_action);
      }
      /**< for turtlebot2 */
      if (robot_base_name_ == kRobotBase_TurtleBot)
      {
        /**< Add LED Action*/
        std::shared_ptr<turtlebot2::LedPolicyAction> led_action = std::make_shared<turtlebot2::LedPolicyAction>();
        std::shared_ptr<turtlebot2::BeeperPolicyAction> beep_action =
                      std::make_shared<turtlebot2::BeeperPolicyAction>();
        if (pname == kCaPolicy_Social)
        {
          led_action->setLedState(kobuki_msgs::Led::ORANGE);
          beep_action->setBeeperState(kobuki_msgs::Sound::CLEANINGSTART);

        }
        else
        {
          led_action->setLedState(kobuki_msgs::Led::GREEN);
          beep_action->setBeeperState(kobuki_msgs::Sound::CLEANINGEND);
        }
        policy->addAction(led_action);
        policy->addAction(beep_action);
      }
      policy_pool_.addPolicy(pname, policy);
    }
  }

  std::string social_source;
  node_handler_.param("social_object_source", social_source, kTopicSocialObjectInFrame);
  vision_obj_sub_ = node_handler_.subscribe(social_source, 10, &CaPolicyRos::onObjectReceived, this);
  ca_policy_pub_ = node_handler_.advertise<ca_policy_msgs::CaPolicy>(kTopicCaPolicy, 1);

  node_handler_.param("max_detection_distance", max_detection_distance_, 2.5);
  node_handler_.param("min_interval", min_interval_, 5.0);
}

void CaPolicyRos::onObjectReceived(const moving_object_msgs::SocialObjectsInFrameConstPtr& msg)
{
  ros::Time now = ros::Time::now();
  bool social = false;

  if (!msg->objects.empty())
  {
    for (auto obj : msg->objects)
    {
      geometry_msgs::Point p = obj.position;
      if ((sqrt(p.x * p.x + p.y * p.y + p.z * p.z) < max_detection_distance_) && obj.name == "person")
      {
        social = true;
        break;
      }
    }
  }

  ca_policy_msgs::CaPolicy pmsg;
  pmsg.header.stamp = now;
  pmsg.header.frame_id = msg->header.frame_id;
  pmsg.robot_id = 0;

  if ((social && policy_pool_.getCurrentPolicy() == kCaPolicy_Social)
      || (!social && policy_pool_.getCurrentPolicy() == kCaPolicy_Normal))
  {
    last_set_time_ = now;
  }
  double duration = now.toSec() - last_set_time_.toSec();
  if (social && policy_pool_.getCurrentPolicy() != kCaPolicy_Social)
  {
    policy_pool_.setCurrentPolicy(kCaPolicy_Social);
    last_set_time_ = now;

    pmsg.id = ca_policy_msgs::CaPolicy::CAPOLICY_PEOPLE;
    pmsg.name = kCaPolicy_Social;
    ca_policy_pub_.publish(pmsg);
  }

  if (!social && policy_pool_.getCurrentPolicy() != kCaPolicy_Normal && duration > min_interval_)
  {
    policy_pool_.setCurrentPolicy(kCaPolicy_Normal);
    last_set_time_ = now;
    pmsg.id = ca_policy_msgs::CaPolicy::CAPOLICY_NORMAL;
    pmsg.name = kCaPolicy_Normal;
    ca_policy_pub_.publish(pmsg);
  }
}

} // namespace
