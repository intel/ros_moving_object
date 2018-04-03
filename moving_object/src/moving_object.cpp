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

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "moving_object/moving_objects.h"

namespace intelligent_ca
{
MovingObjects::MovingObjects(const ros::NodeHandle& nh) : nh_(nh)
{
  frames_.clear();
  nh_.param("max_frames", max_frames_, kDefaultMaxFrames);
  nh_.param("velocity_enabled", velocity_enabled, true);
  nh_.param("fixed_frame", fixed_frame_, std::string("/map"));
}

MovingObjects::MovingObjects() : nh_("~")
{
  frames_.clear();
  nh_.param("max_frames", max_frames_, kDefaultMaxFrames);
  nh_.param("velocity_enabled", velocity_enabled, true);
  nh_.param("fixed_frame", fixed_frame_, std::string("/map"));
}

MovingObjects::~MovingObjects()
{
}

void MovingObjects::calcVelocity(std::shared_ptr<MovingObjectFrame>& frame)
{
  if (!velocity_enabled)
    return;

  unsigned int size_frames = frames_.size();
  ROS_INFO("Caculating VELOCITY (total %d frames...", size_frames);
  for (MovingObjectVector::iterator ob = frame->getMovingObjects().begin(); ob != frame->getMovingObjects().end(); ++ob)
  {
    geometry_msgs::Point sum_vel;
    int sum_count = 0;
    sum_vel.x = sum_vel.y = sum_vel.z = 0.0;

    /**< Find the latest objects from frames (in reverse order) */
    for (unsigned int i = size_frames; i > 0; --i)
    {
      double duration = frames_[i - 1]->getStamp().toSec() - frame->getStamp().toSec();
      if (duration == 0.0)
      {
        continue;
      }

      MovingObject out;
      if (frames_[i - 1]->findMovingObjectById(ob->id, out))
      {
        geometry_msgs::Point32 from = MovingObjectFrame::getCentroid(*ob);
        geometry_msgs::Point32 to = MovingObjectFrame::getCentroid(out);

        /**< Align data's frame id and time. */
        geometry_msgs::PointStamped pt, opt;
        pt.point.x = to.x;
        pt.point.y = to.y;
        pt.point.z = to.z;
        pt.header.frame_id = frame->getTfFrameId();
        pt.header.stamp = frames_[i - 1]->getStamp();
        tf_.transformPoint(frame->getTfFrameId(), frame->getStamp(), pt, fixed_frame_, opt);

        double distance_x = to.x - from.x;
        double distance_y = to.y - from.y;
        double distance_z = to.z - from.z;

        /**< @todo, double check it is set correctly */
        sum_vel.x += distance_x / duration;
        sum_vel.y += distance_y / duration;
        sum_vel.z += distance_z / duration;
        sum_count++;
        break;
      }
    }
    /**< @todo, double check it is set correctly */
    if (sum_count > 0)
    {
      ob->velocity.x = sum_vel.x / sum_count;
      ob->velocity.y = sum_vel.y / sum_count;
      ob->velocity.z = sum_vel.z / sum_count;
    }
  }
}

void MovingObjects::clearOldFrames()
{
  int olds = frames_.size() - max_frames_;

  if (olds > 0)
  {
    frames_.erase(frames_.begin(), frames_.begin() + olds);
  }
}

void MovingObjects::processFrame(const object_msgs::ObjectsInBoxesConstPtr& detect,
                                 const object_analytics_msgs::TrackedObjectsConstPtr& track,
                                 const object_analytics_msgs::ObjectsInBoxes3DConstPtr& loc)
{
  /**< make sure old frames are already cleared first. */
  clearOldFrames();

  ros::Time stamp = detect->header.stamp;
  std::string frame_id = detect->header.frame_id;

  // MovingObjectFrame new_frame(stamp, frame_id, nh_);
  std::shared_ptr<MovingObjectFrame> new_frame = std::make_shared<MovingObjectFrame>(stamp, frame_id, nh_);

  if (loc->objects_in_boxes.size() != 0 && track->tracked_objects.size() != 0 && detect->objects_vector.size() != 0)
  {
    new_frame->addVector(detect->objects_vector);
    new_frame->addVector(track->tracked_objects);
    new_frame->addVector(loc->objects_in_boxes);
    calcVelocity(new_frame);

    frames_.push_back(new_frame);
  }

  new_frame->publish();
}

std::shared_ptr<MovingObjectFrame> MovingObjects::getInstance(const ros::Time stamp, const std::string frame_id)
{
  ROS_INFO("Finding frame: FrameID=%s, Stamp=%10.8f", frame_id.c_str(), stamp.toSec());
  try
  {
    int size = frames_.size();
    for (int i = 0; i < size; ++i)
    {
      if (frames_[i]->getTfFrameId() == frame_id && frames_[i]->getStamp() == stamp)
      {
        return frames_[i];
      }
    }

    std::shared_ptr<MovingObjectFrame> new_frame = std::make_shared<MovingObjectFrame>(stamp, frame_id, nh_);
    frames_.push_back(new_frame);

    return new_frame;
  }
  catch (...)
  {
    ROS_WARN("Failed when getInstance from MovingObjects");
    std::shared_ptr<MovingObjectFrame> new_frame = std::make_shared<MovingObjectFrame>(stamp, frame_id, nh_);
    frames_.push_back(new_frame);
    return new_frame;
  }
}

std::shared_ptr<MovingObjectFrame> MovingObjects::findObjectFrame(const ros::Time stamp, const std::string frame_id)
{
  int size = frames_.size();

  for (int i = 0; i < size; ++i)
  {
    if (frames_[i]->getTfFrameId() == frame_id && frames_[i]->getStamp() == stamp)
    {
      return frames_[i];
    }
  }

  return nullptr;
}

}  // namespace
