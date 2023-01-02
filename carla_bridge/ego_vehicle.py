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


import math


class EgoVehicle(Vehicle):
  def __init__(self, uid, name, parent, node, carla_actor, vehicle_control_applied_callback):
    super(EgoVehicle, self).__init__(uid=uid,
                                     name=name,
                                     parent=parent,
                                     node=node,
                                     carla_actor=carla_actor)
    self.vehicle_info_published = False
    self.vehicle_control_override = False
    self._vehicle_control_applied_callback = vehicle_control_applied_callback

    self.vehicle_status_publisher = node.create_writer(
      self.get_topic_prefix() + "/vehicle_status",
      CarlaEgoVehicleStatus,
      10)

    self.vehicle_info_publisher = node.create_writer(
      self.get_topic_prefix() + "/vehicle_info",
      CarlaEgoVehicleInfo,
      10)

    self.control_subscriber = node.create_reader(
      self.get_topic_prefix() + "/vehicle_control_cmd",
      CarlaEgoVehicleControl,
      lambda data: self.control_command_updated(data, manual_override=False),
    )

    self.manual_control_subscriber = node.create_reader(
      self.get_topic_prefix() + "/vehicle_control_cmd_manual",
      CarlaEgoVehicleControl,
      lambda data: self.control_command_updated(data, manual_override=True),
    )

    self.control_override_subscriber = node.create_reader(
      self.get_topic_prefix() + "/vehicle_control_manual_override",
      Bool,
      self.control_command_override,
    )

    self.enable_autopilot_subscriber = node.create_reader(
      self.get_topic_prefix() + "/enable_autopilot",
      Bool,
      self.enable_autopilot_updated,
    )

  def get_marker_color(self):
    color = ColorRGBA()
    color.r = 0.0
    color.g = 255.0
    color.b = 0.0
    return color

  def send_vehicle_msgs(self, frame, timestamp):
    vehicle_status = CarlaEgoVehicleStatus(
            header=self.get_msg_header("map", timestamp=timestamp))
    vehicle_status.velocity = self.get_vehicle_speed_abs(self.carla_actor)
    vehicle_status.acceleration.linear = self.get_current_ros_accel().linear
    vehicle_status.orientation = self.get_current_ros_pose().orientation
    vehicle_status.control.throttle = self.carla_actor.get_control().throttle
    vehicle_status.control.steer = self.carla_actor.get_control().steer
    vehicle_status.control.brake = self.carla_actor.get_control().brake
    vehicle_status.control.hand_brake = self.carla_actor.get_control().hand_brake
    vehicle_status.control.reverse = self.carla_actor.get_control().reverse
    vehicle_status.control.gear = self.carla_actor.get_control().gear
    vehicle_status.control.manual_gear_shift = self.carla_actor.get_control().manual_gear_shift
    self.vehicle_status_publisher.write(vehicle_status)

    if not self.vehicle_info_published:
      self.vehicle_info_published = True
      vehicle_info = CarlaEgoVehicleInfo()
      vehicle_info.id = self.carla_actor.id
      vehicle_info.type = self.carla_actor.type_id
      vehicle_info.rolename = self.carla_actor.attributes.get('role_name')
      vehicle_physics = self.carla_actor.get_physics_control()

      for wheel in vehicle_physics.wheels:
        wheel_info = CarlaEgoVehicleInfoWheel()
        wheel_info.tire_friction = wheel.tire_friction
        wheel_info.damping_rate = wheel.damping_rate
        wheel_info.max_steer_angle = math.radians(wheel.max_steer_angle)
        wheel_info.radius = wheel.radius
        wheel_info.max_brake_torque = wheel.max_brake_torque
        wheel_info.max_handbrake_torque = wheel.max_handbrake_torque

        inv_T = numpy.array(self.carla_actor.get_transform().get_inverse_matrix(), dtype=float)
        wheel_pos_in_map = numpy.array([wheel.position.x/100.0,
                                wheel.position.y/100.0,
                                wheel.position.z/100.0,
                                1.0])
        wheel_pos_in_ego_vehicle = numpy.matmul(inv_T, wheel_pos_in_map)
        wheel_info.position.x = wheel_pos_in_ego_vehicle[0]
        wheel_info.position.y = -wheel_pos_in_ego_vehicle[1]
        wheel_info.position.z = wheel_pos_in_ego_vehicle[2]
        vehicle_info.wheels.append(wheel_info)

      vehicle_info.max_rpm = vehicle_physics.max_rpm
      vehicle_info.max_rpm = vehicle_physics.max_rpm
      vehicle_info.moi = vehicle_physics.moi
      vehicle_info.damping_rate_full_throttle = vehicle_physics.damping_rate_full_throttle
      vehicle_info.damping_rate_zero_throttle_clutch_engaged = \
          vehicle_physics.damping_rate_zero_throttle_clutch_engaged
      vehicle_info.damping_rate_zero_throttle_clutch_disengaged = \
          vehicle_physics.damping_rate_zero_throttle_clutch_disengaged
      vehicle_info.use_gear_autobox = vehicle_physics.use_gear_autobox
      vehicle_info.gear_switch_time = vehicle_physics.gear_switch_time
      vehicle_info.clutch_strength = vehicle_physics.clutch_strength
      vehicle_info.mass = vehicle_physics.mass
      vehicle_info.drag_coefficient = vehicle_physics.drag_coefficient
      vehicle_info.center_of_mass.x = vehicle_physics.center_of_mass.x
      vehicle_info.center_of_mass.y = vehicle_physics.center_of_mass.y
      vehicle_info.center_of_mass.z = vehicle_physics.center_of_mass.z

      self.vehicle_info_publisher.write(vehicle_info)

  def update(self, frame, timestamp):
    self.send_vehicle_msgs(frame, timestamp)
    super(EgoVehicle, self).update(frame, timestamp)

  def destroy(self):
    pass

  def control_command_override(self, enable):
    self.vehicle_control_override = enable.data

  def control_command_updated(self, ros_vehicle_control, manual_override):
    if manual_override == self.vehicle_control_override:
      vehicle_control = VehicleControl()
      vehicle_control.hand_brake = ros_vehicle_control.hand_brake
      vehicle_control.brake = ros_vehicle_control.brake
      vehicle_control.steer = ros_vehicle_control.steer
      vehicle_control.throttle = ros_vehicle_control.throttle
      vehicle_control.reverse = ros_vehicle_control.reverse
      vehicle_control.manual_gear_shift = ros_vehicle_control.manual_gear_shift
      vehicle_control.gear = ros_vehicle_control.gear
      self.carla_actor.apply_control(vehicle_control)
      self._vehicle_control_applied_callback(self.get_id())

  def enable_autopilot_updated(self, enable_auto_pilot):
    logging.debug("Ego vehicle: Set autopilot to {}".format(enable_auto_pilot.data))
    self.carla_actor.set_autopilot(enable_auto_pilot.data)

  @staticmethod
  def get_vector_length_squared(carla_vector):
    return carla_vector.x * carla_vector.x + \
            carla_vector.y * carla_vector.y + \
            carla_vector.z * carla_vector.z

  @staticmethod
  def get_vehicle_speed_squared(carla_vehicle):
    return EgoVehicle.get_vector_length_squared(carla_vehicle.get_velocity())

  @staticmethod
  def get_vehicle_speed_abs(carla_vehicle):
    speed = math.sqrt(EgoVehicle.get_vehicle_speed_squared(carla_vehicle))
    return speed
