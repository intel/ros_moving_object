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

Header header
uint8 CAPOLICY_NORMAL=0
uint8 CAPOLICY_PEOPLE=1
uint8 CAPOLICY_ROBOT=2

uint8 id
string name
uint32 robot_id  # identifying the robot publishing the topic
                 # in case to multiple robots are sharing data.
                 # robot_id==0 means the self robot.

