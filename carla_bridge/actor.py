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

import carla_bridge.common.transforms as trans

from carla_bridge.pseudo_actor import PseudoActor


class Actor(PseudoActor):
  def __init__(self, uid, name, parent, node, carla_actor):
    super(Actor, self).__init__(uid=uid,
                                name=name,
                                parent=parent,
                                node=node)
    self.carla_actor = carla_actor

  def destroy(self):
    self.carla_actor = None
    super(Actor, self).destroy()

  def get_current_pose(self):
    return trans.carla_transform_to_pose(
        self.carla_actor.get_transform())

  def get_current_transform(self):
    return trans.carla_transform_to_transform(
        self.carla_actor.get_transform())

  def get_current_twist_rotated(self):
    return trans.carla_velocity_to_twist(
            self.carla_actor.get_velocity(),
            self.carla_actor.get_angular_velocity(),
            self.carla_actor.get_transform().rotation)

  def get_current_twist(self):
    return trans.carla_velocity_to_twist(
            self.carla_actor.get_velocity(),
            self.carla_actor.get_angular_velocity())

  def get_current_accel(self):
    return trans.carla_acceleration_to_accel(
            self.carla_actor.get_acceleration())

  def get_id(self):
    return self.carla_actor.id
