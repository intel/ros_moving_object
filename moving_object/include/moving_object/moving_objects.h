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
#ifndef ICA_MOVING_OBJECT_H
#define ICA_MOVING_OBJECT_H

#include <geometry_msgs/Point32.h>
#include <math.h>
#include <ros/ros.h>
#include <vector>
#include <tf/transform_listener.h>

#include "object_frame.h"

namespace intelligent_ca
{
constexpr int kDefaultMaxFrames = 20;

class MovingObjects
{
public:
  MovingObjects();
  MovingObjects(const ros::NodeHandle& nh);
  virtual ~MovingObjects();

  /** @brief Callback function for message filters interface.
     *  @param[in] detect   detection messages received from vision object component.
     *  @param[in] track    tracking messages received from vision object component.
     *  @param[in] loc      localization messages received from vision object component.
     */
  void processFrame(const object_msgs::ObjectsInBoxesConstPtr& detect,
                    const object_analytics_msgs::TrackedObjectsConstPtr& track,
                    const object_analytics_msgs::ObjectsInBoxes3DConstPtr& loc);

  /** @brief Calculates velocity for a given Object Frame.
   *  @param[in+out] frame The frame to be calculated.
   */
  void calcVelocity(std::shared_ptr<MovingObjectFrame>& frame);

  /** @brief Search and return the instance of a frame by the given frame_id and time stamp.
   *         If no cached instance, create a new one and return it.
   *  @param[in] stamp    The time stamp when the frame is taken, which is used to search the frame.
   *  @param[in] frame_id The id the frame, which is used to search the frame.
   *  @return the pointer to the frame instance.
   */
  std::shared_ptr<MovingObjectFrame> getInstance(const ros::Time stamp, const std::string frame_id);

  /** @brief Find and return the object frame by the given time stamp and frame id.
   *  @param[in]  stamp     The time stamp when the frame is taken, which is used to search the frame.
   *  @param[in]  frame_id  The id the frame, which is used to search the frame.
   *  @return     the shared pointer of found MovingObjectFrame if it is found, otherwise nullptr.
   */
  std::shared_ptr<MovingObjectFrame> findObjectFrame(const ros::Time stamp, const std::string frame_id);

  /** @brief Clean the cached frames and remove the old ones if the size of frames is over the threshold. */
  void clearOldFrames();

private:
  std::vector<std::shared_ptr<MovingObjectFrame> > frames_;
  int max_frames_; /**< The number of frames to be archived in memory. */
  bool velocity_enabled;
  std::string fixed_frame_;
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
};

}  // namespace
#endif
