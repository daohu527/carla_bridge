#!/usr/bin/env python

# Copyright 2022 daohu527 <daohu527@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


class Vehicle(TrafficParticipant):
  def __init__(self, uid, name, parent, node, carla_actor):
    self.classification = Object.CLASSIFICATION_CAR
    if 'object_type' in carla_actor.attributes:
      if carla_actor.attributes['object_type'] == 'car':
        self.classification = Object.CLASSIFICATION_CAR
      elif carla_actor.attributes['object_type'] == 'bike':
        self.classification = Object.CLASSIFICATION_BIKE
      elif carla_actor.attributes['object_type'] == 'motorcycle':
        self.classification = Object.CLASSIFICATION_MOTORCYCLE
      elif carla_actor.attributes['object_type'] == 'truck':
        self.classification = Object.CLASSIFICATION_TRUCK
      elif carla_actor.attributes['object_type'] == 'other':
        self.classification = Object.CLASSIFICATION_OTHER_VEHICLE

    super(Vehicle, self).__init__(uid=uid,
                                  name=name,
                                  parent=parent,
                                  node=node,
                                  carla_actor=carla_actor)

  def get_marker_color(self):
    color = ColorRGBA()
    color.r = 255.0
    color.g = 0.0
    color.b = 0.0
    return color

  def get_marker_pose(self):
    extent = self.carla_actor.bounding_box.extent
    marker_transform = self.carla_actor.get_transform()
    marker_transform.location += marker_transform.get_up_vector() * extent.z
    return trans.carla_transform_to_ros_pose(marker_transform)

  def get_classification(self):
    return self.classification
