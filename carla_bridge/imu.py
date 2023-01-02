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

class ImuSensor(Sensor):
  def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
    super(ImuSensor, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode)
    self.imu_publisher = node.create_writer(self.get_topic_prefix(), Imu, 10)

  def destroy(self):
    pass

  def sensor_data_updated(self, carla_imu_measurement):
    imu_msg = Imu()
    imu_msg.header = self.get_msg_header(timestamp=carla_imu_measurement.timestamp)

    imu_msg.angular_velocity.x = -carla_imu_measurement.gyroscope.x
    imu_msg.angular_velocity.y = carla_imu_measurement.gyroscope.y
    imu_msg.angular_velocity.z = -carla_imu_measurement.gyroscope.z

    imu_msg.linear_acceleration.x = carla_imu_measurement.accelerometer.x
    imu_msg.linear_acceleration.y = -carla_imu_measurement.accelerometer.y
    imu_msg.linear_acceleration.z = carla_imu_measurement.accelerometer.z

    roll, pitch, yaw = trans.carla_rotation_to_RPY(carla_imu_measurement.transform.rotation)
    quat = euler2quat(roll, pitch, yaw)
    imu_msg.orientation.w = quat[0]
    imu_msg.orientation.x = quat[1]
    imu_msg.orientation.y = quat[2]
    imu_msg.orientation.z = quat[3]

    self.imu_publisher.write(imu_msg)
