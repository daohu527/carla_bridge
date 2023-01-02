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

from carla_bridge.actor import Actor

class TrafficParticipant(Actor):
  def __init__(self, uid, name, parent, node, carla_actor):
    self.classification_age = 0
    super(TrafficParticipant, self).__init__(uid=uid,
                                             name=name,
                                             parent=parent,
                                             node=node,
                                             carla_actor=carla_actor)

  def update(self, frame, timestamp):
    self.classification_age += 1
    super(TrafficParticipant, self).update(frame, timestamp)

  def get_object_info(self):
    obj = Object(header=self.get_msg_header("map"))
    obj.id = self.get_id()
    obj.pose = self.get_current_ros_pose()
    obj.twist = self.get_current_ros_twist()
    obj.accel = self.get_current_ros_accel()
    obj.shape.type = SolidPrimitive.BOX
    obj.shape.dimensions.extend([
      self.carla_actor.bounding_box.extent.x * 2.0,
      self.carla_actor.bounding_box.extent.y * 2.0,
      self.carla_actor.bounding_box.extent.z * 2.0])

    if self.get_classification() != Object.CLASSIFICATION_UNKNOWN:
      obj.object_classified = True
      obj.classification = self.get_classification()
      obj.classification_certainty = 255
      obj.classification_age = self.classification_age
    return obj

  def get_classification(self):
    return Object.CLASSIFICATION_UNKNOWN

  def get_marker_color(self):
    color = ColorRGBA()
    color.r = 0.
    color.g = 0.
    color.b = 255.
    return color

  def get_marker_pose(self):
    return trans.carla_transform_to_ros_pose(self.carla_actor.get_transform())

  def get_marker(self, timestamp=None):
    marker = Marker(header=self.get_msg_header(frame_id="map", timestamp=timestamp))
    marker.color = self.get_marker_color()
    marker.color.a = 0.3
    marker.id = self.get_id()
    marker.type = Marker.CUBE

    marker.pose = self.get_marker_pose()
    marker.scale.x = self.carla_actor.bounding_box.extent.x * 2.0
    marker.scale.y = self.carla_actor.bounding_box.extent.y * 2.0
    marker.scale.z = self.carla_actor.bounding_box.extent.z * 2.0
    return marker
