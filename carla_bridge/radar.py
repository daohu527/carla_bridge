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

import numpy as np

class Radar(Sensor):
  def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
    super(Radar, self).__init__(uid=uid,
                                name=name,
                                parent=parent,
                                relative_spawn_pose=relative_spawn_pose,
                                node=node,
                                carla_actor=carla_actor,
                                synchronous_mode=synchronous_mode)
    self.radar_publisher = node.create_writer(self.get_topic_prefix(),
                                              PointCloud2, 10)

  def destroy(self):
    pass

  def sensor_data_updated(self, carla_radar_measurement):
    fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='Range', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='Velocity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='AzimuthAngle', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='ElevationAngle', offset=28, datatype=PointField.FLOAT32, count=1)]

    points = []
    for detection in carla_radar_measurement:
      points.append([detection.depth * np.cos(detection.azimuth) * np.cos(-detection.altitude),
                     detection.depth * np.sin(-detection.azimuth) *
                     np.cos(detection.altitude),
                     detection.depth * np.sin(detection.altitude),
                     detection.depth, detection.velocity, detection.azimuth, detection.altitude])
    radar_msg = create_cloud(self.get_msg_header(
            timestamp=carla_radar_measurement.timestamp), fields, points)
    self.radar_publisher.write(radar_msg)
