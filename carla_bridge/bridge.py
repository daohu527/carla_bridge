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

from threading import Thread, Lock, Event

try:
  import queue
except ImportError:
  import Queue as queue

import carla
import cyber_time

class CarlaCyberBridge():
  with open(os.path.join(os.path.dirname(__file__), "CARLA_VERSION")) as f:
    CARLA_VERSION = f.read()[:-1]

  VEHICLE_CONTROL_TIMEOUT = 1.

  def __init__(self):
    super(CarlaCyberBridge, self).__init__("cyber_bridge_node")

  def initialize_bridge(self, carla_world, params):
    self.carla_world = carla_world
    self.parameters = params

    self.cyber_timestamp = cyber_time.Time.now()
    # self.callback_group

    self.synchronous_mode_update_thread = None
    self.shutdown = Event()

    self.carla_settings = carla_world.get_settings()
    if not self.parameters["passive"]:
      # workaround: settings can only applied within non-sync mode
      if self.carla_settings.synchronous_mode:
        self.carla_settings.synchronous_mode = False
        carla_world.apply_settings(self.carla_settings)

        self.loginfo("synchronous_mode: {}".format(
            self.parameters["synchronous_mode"]))
        self.carla_settings.synchronous_mode = self.parameters["synchronous_mode"]
        self.loginfo("fixed_delta_seconds: {}".format(
            self.parameters["fixed_delta_seconds"]))
        self.carla_settings.fixed_delta_seconds = self.parameters["fixed_delta_seconds"]
        carla_world.apply_settings(self.carla_settings)

      self.loginfo("Parameters:")
      for key in self.parameters:
        self.loginfo("  {}: {}".format(key, self.parameters[key]))

      self.sync_mode = self.carla_settings.synchronous_mode and not self.parameters["passive"]
      if self.carla_settings.synchronous_mode and self.parameters["passive"]:
        self.loginfo(
            "Passive mode is enabled and CARLA world is configured in synchronous mode. This configuration requires another client ticking the CARLA world.")

      self.carla_control_queue = queue.Queue()

      # actor factory
      # self.actor_factory = ActorFactory(self, carla_world, self.sync_mode)

      # add world info
      self.world_info = WorldInfo(carla_world=self.carla_world, node=self)
      # add debug helper
      # self.debug_helper = DebugHelper(carla_world.debug, self)

      # Communication topics
      self.clock_publisher = self.create_writer('clock', Clock, 10)

      self.status_publisher = CarlaStatusPublisher(
          self.carla_settings.synchronous_mode,
          self.carla_settings.fixed_delta_seconds,
          self)

      self._all_vehicle_control_commands_received = Event()
      self._expected_ego_vehicle_control_command_ids = []
      self._expected_ego_vehicle_control_command_ids_lock = Lock()

      self.carla_weather_subscriber = \
          self.create_reader("/carla/weather_control", CarlaWeatherParameters,
                            self.on_weather_changed)


    def on_weather_changed(self, weather_parameters):
      """
      Callback on new weather parameters
      :return:
      """
      if not self.carla_world:
          return
      self.loginfo("Applying weather parameters...")
      weather = carla.WeatherParameters()
      weather.cloudiness = weather_parameters.cloudiness
      weather.precipitation = weather_parameters.precipitation
      weather.precipitation_deposits = weather_parameters.precipitation_deposits
      weather.wind_intensity = weather_parameters.wind_intensity
      weather.fog_density = weather_parameters.fog_density
      weather.fog_distance = weather_parameters.fog_distance
      weather.wetness = weather_parameters.wetness
      weather.sun_azimuth_angle = weather_parameters.sun_azimuth_angle
      weather.sun_altitude_angle = weather_parameters.sun_altitude_angle
      self.carla_world.set_weather(weather)
