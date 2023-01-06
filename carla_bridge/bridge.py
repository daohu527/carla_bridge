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

import os
import sys
import logging

try:
  import queue
except ImportError:
  import Queue as queue

from distutils.version import LooseVersion

import carla
import cyber_time
import cyber

from carla_bridge.compatible_node import CompatibleNode
from carla_bridge.actor_factory import ActorFactory
from carla_bridge.world_info import WorldInfo
from carla_bridge.debug_helper import DebugHelper
from carla_bridge.carla_status_publisher import CarlaStatusPublisher


class CarlaCyberBridge(CompatibleNode):
  with open(os.path.join(os.path.dirname(__file__), "CARLA_VERSION")) as f:
    CARLA_VERSION = f.read()[:-1]

  VEHICLE_CONTROL_TIMEOUT = 1.

  def __init__(self):
    super(CarlaCyberBridge, self).__init__()

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

        logging.info("synchronous_mode: {}".format(
            self.parameters["synchronous_mode"]))
        self.carla_settings.synchronous_mode = self.parameters["synchronous_mode"]
        logging.info("fixed_delta_seconds: {}".format(
            self.parameters["fixed_delta_seconds"]))
        self.carla_settings.fixed_delta_seconds = self.parameters["fixed_delta_seconds"]
        carla_world.apply_settings(self.carla_settings)

      logging.info("Parameters:")
      for key in self.parameters:
        logging.info("  {}: {}".format(key, self.parameters[key]))

      self.sync_mode = self.carla_settings.synchronous_mode and not self.parameters["passive"]
      if self.carla_settings.synchronous_mode and self.parameters["passive"]:
        logging.info(
            "Passive mode is enabled and CARLA world is configured in synchronous mode. This configuration requires another client ticking the CARLA world.")

      self.carla_control_queue = queue.Queue()

      # actor factory
      self.actor_factory = ActorFactory(self, carla_world, self.sync_mode)

      # add world info
      self.world_info = WorldInfo(carla_world, self)
      # add debug helper
      self.debug_helper = DebugHelper(carla_world.debug, self)

      # Communication topics
      self.clock_publisher = self.node.create_writer('clock', Clock, 10)

      self.status_publisher = CarlaStatusPublisher(
          self.carla_settings.synchronous_mode,
          self.carla_settings.fixed_delta_seconds,
          self)

      self._all_vehicle_control_commands_received = Event()
      self._expected_ego_vehicle_control_command_ids = []
      self._expected_ego_vehicle_control_command_ids_lock = Lock()

      self.carla_weather_subscriber = self.node.create_reader(
          "/carla/weather_control",
          CarlaWeatherParameters,
          self.on_weather_changed)


  def on_weather_changed(self, weather_parameters):
    """
    Callback on new weather parameters
    :return:
    """
    if not self.carla_world:
        return
    logging.info("Applying weather parameters...")
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

  def process_run_state(self):
    command = None
    while not self.carla_control_queue.empty():
      command = self.carla_control_queue.get()

    while command is not None and cyber.ok():
      self.carla_run_state = command
      if self.carla_run_state == CarlaControl.PAUSE:
        logging.info("State set to PAUSED")
        self.status_publisher.set_synchronous_mode_running(False)
        command = self.carla_control_queue.get()
      elif self.carla_run_state == CarlaControl.PLAY:
        logging.info("State set to PLAY")
        self.status_publisher.set_synchronous_mode_running(True)
        return
      elif self.carla_run_state == CarlaControl.STEP_ONCE:
        logging.info("Execute single step.")
        self.status_publisher.set_synchronous_mode_running(True)
        self.carla_control_queue.put(CarlaControl.PAUSE)
        return

  def _synchronous_mode_update(self):
    while not self.shutdown.is_set() and cyber.ok():
      self.process_run_state()
      if self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
        self._expected_ego_vehicle_control_command_ids = []
        with self._expected_ego_vehicle_control_command_ids_lock:
          for actor_id, actor in self.actor_factory.actors.items():
            if isinstance(actor, EgoVehicle):
              self._expected_ego_vehicle_control_command_ids.append(actor_id)
      self.actor_factory.update_available_objects()
      frame = self.carla_world.tick()

      world_snapshot = self.carla_world.get_snapshot()

      self.status_publisher.set_frame(frame)
      self.update_clock(world_snapshot.timestamp)
      logging.debug("Tick for frame {} returned. Waiting for sensor data...".format(
                frame))
      self._update(frame, world_snapshot.timestamp.elapsed_seconds)
      logging.debug("Waiting for sensor data finished.")

      if self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
        if self._expected_ego_vehicle_control_command_ids:
          if not self._all_vehicle_control_commands_received.wait(CarlaCyberBridge.VEHICLE_CONTROL_TIMEOUT):
            logging.warn("Timeout ({}s) while waiting for vehicle control commands. "
                                     "Missing command from actor ids {}".format(CarlaCyberBridge.VEHICLE_CONTROL_TIMEOUT,
                                                                                self._expected_ego_vehicle_control_command_ids))
          self._all_vehicle_control_commands_received.clear()

  def _carla_time_tick(self, carla_snapshot):
    if not self.shutdown.is_set():
      if self.timestamp_last_run < carla_snapshot.timestamp.elapsed_seconds:
        self.timestamp_last_run = carla_snapshot.timestamp.elapsed_seconds
        self.update_clock(carla_snapshot.timestamp)
        self.status_publisher.set_frame(carla_snapshot.frame)
        self._update(carla_snapshot.frame,
                      carla_snapshot.timestamp.elapsed_seconds)

  def _update(self, frame, timestamp):
    self.world_info.update(frame, timestamp)
    self.actor_factory.update_actor_states(frame, timestamp)

  def _ego_vehicle_control_applied_callback(self, ego_vehicle_id):
    if not self.sync_mode or \
        not self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
      return
    with self._expected_ego_vehicle_control_command_ids_lock:
      if ego_vehicle_id in self._expected_ego_vehicle_control_command_ids:
        self._expected_ego_vehicle_control_command_ids.remove(
                    ego_vehicle_id)
      else:
        logging.warn(
                    "Unexpected vehicle control command received from {}".format(ego_vehicle_id))
      if not self._expected_ego_vehicle_control_command_ids:
        self._all_vehicle_control_commands_received.set()

  def update_clock(self, carla_timestamp):
    if cyber.ok():
      self.cyber_timestamp = cyber.Time(carla_timestamp.elapsed_seconds * 1e9)
      self.clock_publisher.publish(Clock(clock=self.cyber_timestamp))

  def destroy(self):
    logging.info("Shutting down...")
    self.shutdown.set()
    if not self.sync_mode:
      if self.on_tick_id:
        self.carla_world.remove_on_tick(self.on_tick_id)
      self.actor_factory.thread.join()
    else:
      self.synchronous_mode_update_thread.join()
    logging.info("Object update finished.")
    self.debug_helper.destroy()
    self.status_publisher.destroy()
    self.destroy_service(self.spawn_object_service)
    self.destroy_service(self.destroy_object_service)
    self.destroy_subscription(self.carla_weather_subscriber)
    self.carla_control_queue.put(CarlaControl.STEP_ONCE)

    for uid in self._registered_actors:
      self.actor_factory.destroy_actor(uid)
    self.actor_factory.update_available_objects()
    self.actor_factory.clear()
    # todo(zero):
    # super(CarlaCyberBridge, self).destroy()


def main(args=None):
  cyber.init("bridge", args=args)

  carla_bridge = None
  carla_world = None
  carla_client = None
  executor = None
  parameters = {}

  executor = cyber.executors.MultiThreadedExecutor()
  carla_bridge = CarlaCyberBridge()
  executor.add_node(carla_bridge)

  # todo(zero)
  # cyber.on_shutdown(carla_bridge.destroy)

  parameters['host'] = carla_bridge.get_param('host', 'localhost')
  parameters['port'] = carla_bridge.get_param('port', 2000)
  parameters['timeout'] = carla_bridge.get_param('timeout', 2)
  parameters['passive'] = carla_bridge.get_param('passive', False)
  parameters['synchronous_mode'] = carla_bridge.get_param('synchronous_mode', True)
  parameters['synchronous_mode_wait_for_vehicle_control_command'] = carla_bridge.get_param(
      'synchronous_mode_wait_for_vehicle_control_command', False)
  parameters['fixed_delta_seconds'] = carla_bridge.get_param('fixed_delta_seconds',
                                                              0.05)
  parameters['register_all_sensors'] = carla_bridge.get_param('register_all_sensors', True)
  parameters['town'] = carla_bridge.get_param('town', 'Town01')
  role_name = carla_bridge.get_param('ego_vehicle_role_name',
                                      ["hero", "ego_vehicle", "hero1", "hero2", "hero3"])
  parameters["ego_vehicle"] = {"role_name": role_name}

  logging.info("Trying to connect to {host}:{port}".format(
      host=parameters['host'], port=parameters['port']))

  try:
    carla_client = carla.Client(
        host=parameters['host'],
        port=parameters['port'])
    carla_client.set_timeout(parameters['timeout'])

    # check carla version
    dist = pkg_resources.get_distribution("carla")
    if LooseVersion(dist.version) != LooseVersion(CarlaCyberBridge.CARLA_VERSION):
      logging.fatal("CARLA python module version {} required. Found: {}".format(
            CarlaCyberBridge.CARLA_VERSION, dist.version))
      sys.exit(1)

    if LooseVersion(carla_client.get_server_version()) != \
      LooseVersion(carla_client.get_client_version()):
      logging.warn(
            "Version mismatch detected: You are trying to connect to a simulator that might be incompatible with this API. Client API version: {}. Simulator API version: {}"
            .format(carla_client.get_client_version(),
                    carla_client.get_server_version()))

    carla_world = carla_client.get_world()

    if "town" in parameters and not parameters['passive']:
      if parameters["town"].endswith(".xodr"):
        logging.info(
              "Loading opendrive world from file '{}'".format(parameters["town"]))
        with open(parameters["town"]) as od_file:
          data = od_file.read()
        carla_world = carla_client.generate_opendrive_world(str(data))
      else:
        if carla_world.get_map().name != parameters["town"]:
          logging.info("Loading town '{}' (previous: '{}').".format(
                parameters["town"], carla_world.get_map().name))
          carla_world = carla_client.load_world(parameters["town"])
      carla_world.tick()

    carla_bridge.initialize_bridge(carla_client.get_world(), parameters)

    carla_bridge.node.spin()

  except (IOError, RuntimeError) as e:
    logging.error("Error: {}".format(e))
  except KeyboardInterrupt:
    pass
  finally:
    cyber.shutdown()
    del carla_world
    del carla_client

if __name__ == "__main__":
  main()
