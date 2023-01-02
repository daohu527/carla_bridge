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


class Sensor(Actor):
  def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode, is_event_sensor=False):
    super(Sensor, self).__init__(uid=uid,
                                 name=name,
                                 parent=parent,
                                 node=node,
                                 carla_actor=carla_actor)
    self.relative_spawn_pose = relative_spawn_pose
    self.synchronous_mode = synchronous_mode
    self.queue = queue.Queue()
    self.next_data_expected_time = None
    self.sensor_tick_time = None
    self.is_event_sensor = is_event_sensor
    self._callback_active = Lock()

    try:
      self.sensor_tick_time = float(carla_actor.attributes["sensor_tick"])
    except:
      self.sensor_tick_time = None

    self._tf_broadcaster = tf2_ros.TransformBroadcaster(node)

  def get_transform(self, pose, timestamp):
    if self.synchronous_mode:
      if not self.relative_spawn_pose:
        return
      pose = self.relative_spawn_pose
      child_frame_id = self.get_prefix()
      if self.parent is not None:
        frame_id = self.parent.get_prefix()
      else:
        frame_id = "map"
    else:
      child_frame_id = self.get_prefix()
      frame_id = "map"

    transform = tf2_ros.TransformStamped()
    transform.header.stamp = roscomp.ros_timestamp(sec=timestamp, from_sec=True)
    transform.header.frame_id = frame_id
    transform.child_frame_id = child_frame_id

    transform.transform.translation.x = pose.position.x
    transform.transform.translation.y = pose.position.y
    transform.transform.translation.z = pose.position.z

    transform.transform.rotation.x = pose.orientation.x
    transform.transform.rotation.y = pose.orientation.y
    transform.transform.rotation.z = pose.orientation.z
    transform.transform.rotation.w = pose.orientation.w

    return transform

  def publish_tf(self, pose, timestamp):
    transform = self.get_transform(pose, timestamp)
    try:
      self._tf_broadcaster.sendTransform(transform)
    except:
      pass

  def listen(self):
    self.carla_actor.listen(self._callback_sensor_data)

  def destroy(self):
    self._callback_active.acquire()
    if self.carla_actor.is_listening:
      self.carla_actor.stop()
    super(Sensor, self).destroy()

  def _callback_sensor_data(self, carla_sensor_data):
    if not self._callback_active.acquire(False):
      return
    if self.synchronous_mode:
      if self.sensor_tick_time:
        self.next_data_expected_time = carla_sensor_data.timestamp + \
                    float(self.sensor_tick_time)
      self.queue.put(carla_sensor_data)
    else:
      self.publish_tf(trans.carla_transform_to_ros_pose(
                carla_sensor_data.transform), carla_sensor_data.timestamp)
      try:
        self.sensor_data_updated(carla_sensor_data)
      except:
        pass
    self._callback_active.release()

  @abstractmethod
  def sensor_data_updated(self, carla_sensor_data):
    pass

  def _update_synchronous_event_sensor(self, frame, timestamp):
    while True:
      try:
        carla_sensor_data = self.queue.get(block=False)
        if carla_sensor_data.frame != frame:
          pass
        self.publish_tf(trans.carla_transform_to_ros_pose(
                    carla_sensor_data.transform), timestamp)
        self.sensor_data_updated(carla_sensor_data)
      except queue.Empty:
        return

  def _update_synchronous_sensor(self, frame, timestamp):
    while not self.next_data_expected_time or \
            (not self.queue.empty() or
             self.next_data_expected_time and
             self.next_data_expected_time < timestamp):
      while True:
        try:
          carla_sensor_data = self.queue.get(timeout=1.0)
          if carla_sensor_data.frame == frame:
            self.publish_tf(trans.carla_transform_to_ros_pose(
                            carla_sensor_data.transform), timestamp)
                            self.sensor_data_updated(carla_sensor_data)
            return
          elif carla_sensor_data.frame < frame:
            pass
        except queue.Empty:
          return

  def update(self, frame, timestamp):
    if self.synchronous_mode:
      if self.is_event_sensor:
        self._update_synchronous_event_sensor(frame, timestamp)
      else:
        self._update_synchronous_sensor(frame, timestamp)
    super(Sensor, self).update(frame, timestamp)


def create_cloud(header, fields, points):
  pass
