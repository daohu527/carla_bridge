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

import math

class ActorControl(PseudoActor):
  def __init__(self, uid, name, parent, node):
    super(ActorControl, self).__init__(uid=uid,
                                       name=name,
                                       parent=parent,
                                       node=node)
    self.set_location_subscriber = self.node.create_reader(
      self.get_topic_prefix() + "/set_transform",
      Pose,
      self.on_pose,
    )

    self.twist_control_subscriber = self.node.create_reader(
      self.get_topic_prefix() + "/set_target_velocity",
      Twist,
      self.on_twist
    )

  def destroy(self):
    # todo(zero):
    super(ActorControl, self).destroy()

  @staticmethod
  def get_blueprint_name(self):
    return "actor.pseudo.control"

  def on_pose(self, pose):
    if self.parent and self.parent.carla_actor.is_alive:
      self.parent.carla_actor.set_transform(trans.ros_pose_to_carla_transform(pose))
      if isinstance(self.parent, Sensor):
        self.parent.relative_spawn_pose = pose

  def on_twist(self, twist):
    if not self.parent.vehicle_control_override:
      angular_velocity = Vector3D()
      angular_velocity.z = math.degrees(twist.angular.z)

      rotation_matrix = trans.carla_rotation_to_numpy_rotation_matrix(
          self.parent.carla_actor.get_transform().rotation)
      linear_vector = numpy.array([twist.linear.x, twist.linear.y, twist.linear.z])
      rotated_linear_vector = rotation_matrix.dot(linear_vector)
      linear_velocity = Vector3D()
      linear_velocity.x = rotated_linear_vector[0]
      linear_velocity.y = -rotated_linear_vector[1]
      linear_velocity.z = rotated_linear_vector[2]

      logging.debug("Set velocity linear: {}, angular: {}".format(
          linear_velocity, angular_velocity))
      self.parent.carla_actor.set_target_velocity(linear_velocity)
      self.parent.carla_actor.set_target_angular_velocity(angular_velocity)
