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

# todo(zero): add carla_msg
from carla_msgs.msg import CarlaStatus

class CarlaStatusPublisher(object):
  """[summary]

  Args:
      object ([type]): [description]
  """
  def __init__(self, synchronous_mode, fixed_delta_seconds, node):
    """[summary]

    Args:
        synchronous_mode ([type]): [description]
        fixed_delta_seconds ([type]): [description]
        node ([type]): [description]
    """
    self.synchronous_mode = synchronous_mode
    self.synchronous_mode_running = True
    self.fixed_delta_seconds = fixed_delta_seconds
    self.node = node

    if self.fixed_delta_seconds is None:
      self.fixed_delta_seconds = 0
    self.frame = 0
    # todo(zero): cyber writer have no callback
    self.carla_status_publisher = \
      node.create_writer("/carla/status", CarlaStatus, 10)
    self.publish()

  def destroy(self):
    """[summary]
    """
    pass

  def publish(self):
    """[summary]
    """
    status_msg = CarlaStatus()
    status_msg.frame = self.frame
    status_msg.synchronous_mode = self.synchronous_mode
    status_msg.synchronous_mode_running = self.synchronous_mode_running
    status_msg.fixed_delta_seconds = self.fixed_delta_seconds
    self.carla_status_publisher.write(status_msg)

  def set_synchronous_mode_running(self, running):
    """[summary]

    Args:
        running ([type]): [description]
    """
    if self.synchronous_mode_running != running:
      self.synchronous_mode_running = running
      self.publish()

  def set_frame(self, frame):
    """[summary]

    Args:
        frame ([type]): [description]
    """
    if self.frame != frame:
      self.frame = frame
      self.publish()
