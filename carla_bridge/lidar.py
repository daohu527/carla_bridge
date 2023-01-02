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

import numpy

class Lidar(Sensor):
  def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
    super(Lidar, self).__init__(uid=uid,
                                name=name,
                                parent=parent,
                                relative_spawn_pose=relative_spawn_pose,
                                node=node,
                                carla_actor=carla_actor,
                                synchronous_mode=synchronous_mode)

    self.lidar_publisher = node.create_writer(self.get_topic_prefix(),
                                              PointCloud2, 10)

  def destroy(self):
    pass

  def sensor_data_updated(self, carla_lidar_measurement):
    header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)
    fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]

    lidar_data = numpy.fromstring(
            bytes(carla_lidar_measurement.raw_data), dtype=numpy.float32)
    lidar_data = numpy.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))
    lidar_data[:, 1] *= -1
    point_cloud_msg = create_cloud(header, fields, lidar_data)
    self.lidar_publisher.write(point_cloud_msg)


class SemanticLidar(Sensor):
  def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
    super(SemanticLidar, self).__init__(uid=uid,
                                        name=name,
                                        parent=parent,
                                        relative_spawn_pose=relative_spawn_pose,
                                        node=node,
                                        carla_actor=carla_actor,
                                        synchronous_mode=synchronous_mode)
    self.semantic_lidar_publisher = node.create_writer(self.get_topic_prefix(),
                                                       PointCloud2, 10)

  def destroy(self):
    pass

  def sensor_data_updated(self, carla_lidar_measurement):
    header = self.get_msg_header(timestamp=carla_lidar_measurement.timestamp)
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='CosAngle', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='ObjIdx', offset=16, datatype=PointField.UINT32, count=1),
        PointField(name='ObjTag', offset=20, datatype=PointField.UINT32, count=1)]
    lidar_data = numpy.fromstring(bytes(carla_lidar_measurement.raw_data),
                                      dtype=numpy.dtype([
                                          ('x', numpy.float32),
                                          ('y', numpy.float32),
                                          ('z', numpy.float32),
                                          ('CosAngle', numpy.float32),
                                          ('ObjIdx', numpy.uint32),
                                          ('ObjTag', numpy.uint32)
                                      ]))
    lidar_data['y'] *= -1
    point_cloud_msg = create_cloud(header, fields, lidar_data.tolist())
    self.semantic_lidar_publisher.write(point_cloud_msg)
