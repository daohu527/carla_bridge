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

from carla import WalkerControl

from carla_msgs.msg import CarlaWalkerControl


class Walker(TrafficParticipant):
  def __init__(self, uid, name, parent, node, carla_actor):
    super(Walker, self).__init__(uid=uid,
                                 name=name,
                                 parent=parent,
                                 node=node,
                                 carla_actor=carla_actor)

    self.control_subscriber = self.node.create_reader(
      self.get_topic_prefix() + "/walker_control_cmd",
      CarlaWalkerControl,
      self.control_command_updated)

  def destroy(self):
    pass

  def control_command_updated(self, cyber_walker_control):
    walker_control = WalkerControl()
    walker_control.direction.x = cyber_walker_control.direction.x
    walker_control.direction.y = -cyber_walker_control.direction.y
    walker_control.direction.z = cyber_walker_control.direction.z
    walker_control.speed = cyber_walker_control.speed
    walker_control.jump = cyber_walker_control.jump
    self.carla_actor.apply_control(walker_control)

  def get_classification(self):
    return Object.CLASSIFICATION_PEDESTRIAN