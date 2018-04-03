# Copyright (c) 2017 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.3)
project(ca_policy)

add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs geometry_msgs
		moving_object_msgs
    ca_policy_msgs
		message_filters
    tf
    kobuki_msgs
		)

# What other packages will need to use this package
# What other packages will need to use this package
catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs 
       moving_object_msgs message_filters tf ca_policy_msgs
       kobuki_msgs
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

# Add_executables
add_executable(ca_policy
  src/ca_policy_node.cpp
  src/ca_policy_ros.cpp
  src/ca_policy.cpp
  src/reconfig_policy_action.cpp
  src/ca_policy_pool.cpp
  src/waterbot/ledbelt_policy_action.cpp
  src/turtlebot/led_policy_action.cpp
  src/turtlebot/beeper_policy_action.cpp
  )
target_link_libraries(ca_policy ${catkin_LIBRARIES})
add_dependencies(ca_policy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
# Mark executables and/or libraries for installation
install(TARGETS ca_policy
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


