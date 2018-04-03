
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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "moving_object/moving_object_ros.h"

namespace intelligent_ca
{
MovingObjectRos::MovingObjectRos() : nh_("~")
{
  ROS_INFO("ENTER default MovingObjectRos Constructor...");

  onInit();
}

MovingObjectRos::MovingObjectRos(ros::NodeHandle& nh) : nh_(nh)
{
  ROS_INFO("ENTER MovingObjectRos Constructor...");

  onInit();
}

void MovingObjectRos::onInit()
{
  nh_.param("msg_object_detection", msg_object_detection_, kTopicObjectDetection);
  nh_.param("msg_object_tracking", msg_object_tracking_, kTopicObjectTracking);
  nh_.param("msg_object_localization", msg_object_localization_, kTopicObjectLocalization);

  f_detection_sub_ = std::unique_ptr<FilteredDetection>(new FilteredDetection(nh_, msg_object_detection_, 1));
  f_tracking_sub_ = std::unique_ptr<FilteredTracking>(new FilteredTracking(nh_, msg_object_tracking_, 1));
  f_localization_sub_ =
      std::unique_ptr<FilteredLocalization>(new FilteredLocalization(nh_, msg_object_localization_, 1));
  sync_sub_ =
      std::unique_ptr<FilteredSync>(new FilteredSync(*f_detection_sub_, *f_tracking_sub_, *f_localization_sub_, 10));
  sync_sub_->registerCallback(boost::bind(&MovingObjectRos::onObjectsReceived, this, _1, _2, _3));

  frames_ = std::make_shared<MovingObjects>(nh_);
  ROS_INFO("message_detction:%s, tracking:%s, localization:%s", msg_object_detection_.c_str(),
           msg_object_tracking_.c_str(), msg_object_localization_.c_str());
}

MovingObjectRos::~MovingObjectRos()
{
}

void MovingObjectRos::onObjectsReceived(const object_msgs::ObjectsInBoxesConstPtr& detect,
                                        const object_analytics_msgs::TrackedObjectsConstPtr& track,
                                        const object_analytics_msgs::ObjectsInBoxes3DConstPtr& loc)
{
  if (loc->header.frame_id != track->header.frame_id || track->header.frame_id != detect->header.frame_id ||
      loc->header.frame_id != detect->header.frame_id)
  {
    return;
  }

  frames_->processFrame(detect, track, loc);
}

}  // namespace
