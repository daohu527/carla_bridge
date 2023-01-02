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


class Gnss(Sensor):
  def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
    super(Gnss, self).__init__(uid=uid,
                               name=name,
                               parent=parent,
                               relative_spawn_pose=relative_spawn_pose,
                               node=node,
                               carla_actor=carla_actor,
                               synchronous_mode=synchronous_mode)
    self.gnss_publisher = node.create_writer(
        self.get_topic_prefix(), NavSatFix, 10)

  def destroy(self):
    pass

  def sensor_data_updated(self, carla_gnss_measurement):
    navsatfix_msg = NavSatFix()
    navsatfix_msg.header = self.get_msg_header(timestamp=carla_gnss_measurement.timestamp)
    navsatfix_msg.latitude = carla_gnss_measurement.latitude
    navsatfix_msg.longitude = carla_gnss_measurement.longitude
    navsatfix_msg.altitude = carla_gnss_measurement.altitude
    self.gnss_publisher.write(navsatfix_msg)
