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

"""
Class to handle the carla map
"""

from carla_msgs.msg import CarlaWorldInfo

class WorldInfo(object):
  """Publish the map

  Args:
      object ([type]): [description]
  """
  def __init__(self, carla_world, node):
    """[summary]

    Args:
        carla_world ([type]): [description]
        node ([type]): [description]
    """
    self._node = node
    self._carla_map = carla_world.get_map()
    self._map_published = False
    self._world_info_publisher = \
      node.create_writer("/carla/world_info", CarlaWorldInfo, 10)

  def destroy(self):
    self._carla_map = None

  def update(self, frame, timestamp):
    """Function (override) to update this object.

    Args:
        frame ([type]): [description]
        timestamp ([type]): [description]
    """
    if not self._map_published:
      open_drive_msg = CarlaWorldInfo()
      open_drive_msg.map_name = self._carla_map.name
      open_drive_msg.opendrive = self._carla_map.to_opendrive()
      self._world_info_publisher.write(open_drive_msg)
      self._map_published = True
