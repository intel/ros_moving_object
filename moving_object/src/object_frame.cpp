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
#include <sensor_msgs/RegionOfInterest.h>
#include <iostream>
#include "moving_object/object_frame.h"
#include <moving_object_msgs/MovingObject.h>
#include <moving_object_msgs/SocialObject.h>
#include <moving_object_msgs/SocialObjectsInFrame.h>

namespace intelligent_ca
{
MovingObjectFrame::MovingObjectFrame() : nh_("/intelligent_ca/"), published_(false)
{
  moving_objects_pub_ = nh_.advertise<moving_object_msgs::MovingObjectsInFrame>(kTopicObjectsInFrame, 1);
  social_object_pub_ = nh_.advertise<moving_object_msgs::SocialObjectsInFrame>(kTopicSocialObjectInFrame, 1);

  initParameter();

  objects_detected_.clear();
  objects_tracked_.clear();
  objects_localized_.clear();

  moving_objects_.clear();
}

MovingObjectFrame::MovingObjectFrame(const ros::NodeHandle nh) : nh_(nh), published_(false)
{
  moving_objects_pub_ = nh_.advertise<moving_object_msgs::MovingObjectsInFrame>(kTopicObjectsInFrame, 1);
  social_object_pub_ = nh_.advertise<moving_object_msgs::SocialObjectsInFrame>(kTopicSocialObjectInFrame, 1);

  initParameter();

  objects_detected_.clear();
  objects_tracked_.clear();
  objects_localized_.clear();

  moving_objects_.clear();
}

MovingObjectFrame::MovingObjectFrame(const ros::Time& stamp, const std::string& frame_id, const ros::NodeHandle nh)
  : nh_(nh), published_(false)
{
  moving_objects_pub_ = nh_.advertise<moving_object_msgs::MovingObjectsInFrame>(kTopicObjectsInFrame, 1);
  social_object_pub_ = nh_.advertise<moving_object_msgs::SocialObjectsInFrame>(kTopicSocialObjectInFrame, 1);

  initParameter();

  objects_detected_.clear();
  objects_tracked_.clear();
  objects_localized_.clear();

  moving_objects_.clear();
  stamp_ = stamp;
  tf_frame_id_ = frame_id;
}
MovingObjectFrame::~MovingObjectFrame()
{
}

void MovingObjectFrame::initParameter()
{
  nh_.param("social_msg_enabled", social_msg_enabled_, true);
  nh_.param("moving_object_msg_enabled", moving_object_msg_enabled_, true);
  nh_.param("posibility_threshold", posibility_threshold_, 0.1);

  is_merging_ = false;

  /**< @todo TODO: get params for social_filter_ */
  social_filter_.clear();
  social_filter_.push_back("person");
}

void MovingObjectFrame::addVector(const DetectionVector& vector)
{
  objects_detected_ = vector;
  mergeObjects();
  ROS_INFO("Add detection vector [size:%lu] ...", objects_detected_.size());
}

void MovingObjectFrame::addVector(const TrackingVector& vector)
{
  objects_tracked_ = vector;
  mergeObjects();
}

void MovingObjectFrame::addVector(const LocalizationVector& vector)
{
  objects_localized_ = vector;
  mergeObjects();
}

void MovingObjectFrame::mergeObjects()
{
  if (is_merging_)
  {
    ROS_INFO("Objects is already merging...");
    return;
  }

  // ROS_INFO("published:%s, data ready:%s", published_ ? "true" : "false", isDataReady() ? "true" : "false");
  if (published_ || !isDataReady())
  {
    //ROS_INFO("Already published or data not ready. Do nothing");
    return;
  }

  is_merging_ = true;
  ROS_INFO("Generating and publishing moving objects...");
  for (DetectionVector::iterator it = objects_detected_.begin(); it != objects_detected_.end(); ++it)
  {
    if (it->object.probability < posibility_threshold_)
    {
      continue;
    }

    ObjectRoi roi = it->roi;
    MovingObject moving_obj;
    TrackingObjectInBox track_obj;
    LocalizationObjectInBox loc_obj;

    bool result = findTrackingObjectByRoi(roi, track_obj);
    if (result)
    {
      result = findLocalizationObjectByRoi(roi, loc_obj);
      if (result)
      {
        moving_obj.min = loc_obj.min;
        moving_obj.max = loc_obj.max;
        moving_obj.id = track_obj.id;
        moving_obj.type = it->object.object_name;
        moving_obj.probability = it->object.probability;
        moving_obj.roi = it->roi;
        moving_obj.velocity.x = moving_obj.velocity.y = moving_obj.velocity.z = -1.0;

        moving_objects_.push_back(moving_obj);
      }
    }

  }  // end of for(...)
  is_merging_ = false;
}

bool MovingObjectFrame::findMovingObjectById(const int id, MovingObject& out)
{
  MovingObjectVector temp_objects = moving_objects_;
  for (auto t : temp_objects)
  {
    if (t.id == id)
    {
      out = t;
      return true;
    }
  }
  return false;
}

bool MovingObjectFrame::findMovingObjectByRoi(const ObjectRoi& roi, MovingObject& out)
{
  MovingObjectVector temp_objects = moving_objects_;
  for (auto t : temp_objects)
  {
    if (roi.x_offset == t.roi.x_offset && roi.y_offset == t.roi.y_offset && roi.width == t.roi.width &&
        roi.height == t.roi.height)
    {
      out = t;
      return true;
    }
  }

  return false;
}

bool MovingObjectFrame::findTrackingObjectByRoi(const ObjectRoi& roi, TrackingObjectInBox& track)
{
  for (auto t : objects_tracked_)
  {
    if (roi.x_offset == t.roi.x_offset && roi.y_offset == t.roi.y_offset && roi.width == t.roi.width &&
        roi.height == t.roi.height)
    {
      track = t;
      return true;
    }
  }

  return false;
}

bool MovingObjectFrame::findLocalizationObjectByRoi(const ObjectRoi& roi, LocalizationObjectInBox& loc)
{
  for (auto t : objects_localized_)
  {
    if (roi.x_offset == t.roi.x_offset && roi.y_offset == t.roi.y_offset && roi.width == t.roi.width &&
        roi.height == t.roi.height)
    {
      loc = t;
      return true;
    }
  }

  return false;
}

bool MovingObjectFrame::publish()
{
  if (published_)
  {
    ROS_INFO("Moving objects have been already published, do nothing");
    return false;
  }

  if (moving_object_msg_enabled_)
  {
    MovingObjectMsg msg;
    msg.header.frame_id = tf_frame_id_;
    msg.header.stamp = stamp_;
    msg.objects = moving_objects_;
    moving_objects_pub_.publish(msg);
  }

  if (social_msg_enabled_)
  {
    SoicalObjectVector socials;
    socials.clear();

    if (!moving_objects_.empty())
    {
      for (auto ob : moving_objects_)
      {
        if (!isSocialObject(ob))
          break;

        SocialObject so;
        so.id = ob.id;
        so.name = ob.type;
        geometry_msgs::Point32 c = getCentroid(ob);
        so.position.x = c.x;
        so.position.y = c.y;
        so.position.z = c.z;
        so.velocity = ob.velocity;
        so.reliability = ob.probability;
        so.tagnames.clear(); /**< Not used */
        so.tags.clear();     /**< Not used */
        socials.push_back(so);
      }
    }

    SocialObjectsInFrameMsg msg;
    msg.header.frame_id = tf_frame_id_;
    msg.header.stamp = stamp_;
    msg.objects = socials;
    social_object_pub_.publish(msg);
  }

  setFlagPublished(true);

  return true;
}

bool MovingObjectFrame::isSocialObject(MovingObject& ob)
{
  for (auto f : social_filter_)
  {
    if (ob.type.find(f) != std::string::npos)
    {
      return true;
    }
  }

  return false;
}

void MovingObjectFrame::setFlagPublished(bool state)
{
  published_ = state;
}
}  // namespace
