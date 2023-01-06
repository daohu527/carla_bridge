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

import carla

from carla_bridge.sensor import Sensor

from modules.drivers.proto.sensor_image_pb2 import Image

class Camera(Sensor):
  cv_bridge = CvBridge()

  def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor,
      synchronous_mode, is_event_sensor=False):
    super(Camera, self).__init__(uid=uid,
                                 name=name,
                                 parent=parent,
                                 relative_spawn_pose=relative_spawn_pose,
                                 node=node,
                                 carla_actor=carla_actor,
                                 synchronous_mode=synchronous_mode,
                                 is_event_sensor=is_event_sensor)
    if self.__class__.__name__ == 'Camera':
      pass
    else:
      self._build_camera_info()

    self.camera_info_publisher = node.create_writer(
        self.get_topic_prefix() + "/camera_info", CameraInfo, 10)
    self.camera_image_publisher = node.create_writer(
        self.get_topic_prefix() + "/image", Image, 10)

  def destroy(self):
    super(Camera, self).destroy()
    # todo(zero): destroy writer

  def _build_camera_info(self):
    camera_info = CameraInfo()
    # todo(zero): add camerainfo

  def sensor_data_updated(self, carla_camera_data):
    img_msg = self.get_image(carla_camera_data)
    cam_info = self._camera_info
    cam_info.header = img_msg.header
    self.camera_info_publisher.write(cam_info)
    self.camera_image_publisher.write(img_msg)

  def get_transform(self, pose, timestamp):
    tf_msg = super(Camera, self).get_transform(pose, timestamp)
    rotation = tf_msg.transform.rotation
    quat = [rotation.w, rotation.x, rotation.y, rotation.z]
    quat_swap = transforms3d.quaternions.mat2quat(
        np.matrix([[0, 0, 1],
                      [-1, 0, 0],
                      [0, -1, 0]]))
    quat = transforms3d.quaternions.qmult(quat, quat_swap)
    tf_msg.transform.rotation.w = quat[0]
    tf_msg.transform.rotation.x = quat[1]
    tf_msg.transform.rotation.y = quat[2]
    tf_msg.transform.rotation.z = quat[3]
    return tf_msg

  def get_image(self, carla_camera_data):
    image_data_array, encoding = self.get_carla_image_data_array(
        carla_camera_data)
    img_msg = Camera.cv_bridge.cv2_to_imgmsg(image_data_array, encoding=encoding)
    img_msg.header = self.get_msg_header(timestamp=carla_camera_data.timestamp)
    return img_msg

  @abstractmethod
  def get_carla_image_data_array(self, carla_camera_data):
    pass


class RgbCamera(Camera):
  def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
    super(RgbCamera, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode)

  def get_carla_image_data_array(self, carla_image):
    carla_image_data_array = np.ndarray(
      shape=(carla_image.height, carla_image.width, 4),
      dtype=np.uint8, buffer=carla_image.raw_data)
    return carla_image_data_array, 'bgra8'


class DepthCamera(Camera):
  def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
    super(DepthCamera, self).__init__(uid=uid,
                                      name=name,
                                      parent=parent,
                                      relative_spawn_pose=relative_spawn_pose,
                                      node=node,
                                      carla_actor=carla_actor,
                                      synchronous_mode=synchronous_mode)

  def get_carla_image_data_array(self, carla_image):
    bgra_image = np.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=np.uint8, buffer=carla_image.raw_data)

    scales = np.array([65536.0, 256.0, 1.0, 0]) / (256**3 - 1) * 1000
    depth_image = np.dot(bgra_image, scales).astype(np.float32)

    return depth_image, 'passthrough'


class SemanticSegmentationCamera(Camera):
  def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
    super(SemanticSegmentationCamera, self).__init__(uid=uid,
                                                     name=name,
                                                     parent=parent,
                                                     relative_spawn_pose=relative_spawn_pose,
                                                     node=node,
                                                     synchronous_mode=synchronous_mode,
                                                     carla_actor=carla_actor)

  def get_carla_image_data_array(self, carla_image):
    carla_image.convert(carla.ColorConverter.CityScapesPalette)
    carla_image_data_array = np.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=np.uint8, buffer=carla_image.raw_data)
    return carla_image_data_array, 'bgra8'

class DVSCamera(Camera):
  def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
    super(DVSCamera, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode,
                                    is_event_sensor=True)
    self._dvs_events = None
    self.dvs_camera_publisher = node.create_writer(
      self.get_topic_prefix() + '/events', PointCloud2, 10)

  def destroy(self):
    # todo(zero): delete dvs_camera_publisher
    pass

  def sensor_data_updated(self, carla_dvs_event_array):
    super(DVSCamera, self).sensor_data_updated(carla_dvs_event_array)
    header = self.get_msg_header(timestamp=carla_dvs_event_array.timestamp)
    fields = [
        PointField(name='x', offset=0, datatype=PointField.UINT16, count=1),
        PointField(name='y', offset=2, datatype=PointField.UINT16, count=1),
        PointField(name='t', offset=4, datatype=PointField.FLOAT64, count=1),
        PointField(name='pol', offset=12, datatype=PointField.INT8, count=1)]

    dvs_events_msg = create_cloud(header, fields, self._dvs_events.tolist())
    self.dvs_camera_publisher.write(dvs_events_msg)

  def get_carla_image_data_array(self, carla_dvs_event_array):
    self._dvs_events = np.frombuffer(carla_dvs_event_array.raw_data,
                                        dtype=np.dtype([
                                            ('x', np.uint16),
                                            ('y', np.uint16),
                                            ('t', np.int64),
                                            ('pol', np.bool)
                                        ]))
    carla_image_data_array = np.zeros(
            (carla_dvs_event_array.height, carla_dvs_event_array.width, 3),
            dtype=np.uint8)
    # Blue is positive, red is negative
    carla_image_data_array[self._dvs_events[:]['y'], self._dvs_events[:]['x'],
                               self._dvs_events[:]['pol'] * 2] = 255

    return carla_image_data_array, 'bgr8'
