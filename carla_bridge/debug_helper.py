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
import logging

import carla


class DebugHelper(object):
  def __init__(self, carla_debug_helper, node):
    self.debug = carla_debug_helper
    self.node = node
    self.marker_subscriber = self.node.create_reader("/carla/debug_marker",
                                                     MarkerArray,
                                                     self.on_marker)

  def destroy(self):
    logging.debug("Destroy DebugHelper")
    self.debug = None
    # todo(zero)

  def on_marker(self, marker_array):
    for marker in marker_array.markers:
      if marker.header.frame_id != "map":
        continue
      lifetime = -1.
      if marker.lifetime:
        lifetime = marker.lifetime.to_sec()
      color = carla.Color(int(marker.color.r * 255), int(marker.color.g * 255),
                                int(marker.color.b * 255), int(marker.color.a * 255))
      if marker.type == Marker.POINTS:
        self.draw_points(marker, lifetime, color)
      elif marker.type == Marker.LINE_STRIP:
        self.draw_line_strips(marker, lifetime, color)
      elif marker.type == Marker.ARROW:
        self.draw_arrow(marker, lifetime, color)
      elif marker.type == Marker.CUBE:
        self.draw_box(marker, lifetime, color)
      else:
        logging.warn(
                    "Marker type '{}' not supported.".format(marker.type))

  def draw_arrow(self, marker, lifetime, color):
    if marker.points:
      if not len(marker.points) == 2:
        logging.warn("Drawing arrow from points requires two points. Received {}".format(
                    len(marker.points)))
        return
      thickness = marker.scale.x
      arrow_size = marker.scale.y
      start = carla.Location(x=marker.points[0].x, y=-marker.points[0].y,
                                   z=marker.points[0].z)
      end = carla.Location(x=marker.points[1].x, y=-marker.points[1].y, z=marker.points[1].z)
      logging.info("Draw Arrow from {} to {} (color: {}, lifetime: {}, "
                              "thickness: {}, arrow_size: {})".format(start, end, color, lifetime,
                                                                      thickness, arrow_size))
      self.debug.draw_arrow(start, end, thickness=thickness, arrow_size=arrow_size,
                                  color=color, life_time=lifetime)
    else:
      logging.warn("Drawing arrow from Position/Orientation not yet supported. "
                        "Please use points.")

  def draw_points(self, marker, lifetime, color):
    for point in marker.points:
      location = carla.Location(x=point.x, y=-point.y, z=point.z)
      size = marker.scale.x
      logging.info("Draw Point {} (color: {}, lifetime: {}, size: {})".format(
                location, color, lifetime, size))
      self.debug.draw_point(location, size=size, color=color, life_time=lifetime)

  def draw_line_strips(self, marker, lifetime, color):
    if len(marker.points) < 2:
      logging.warn("Drawing line-strip requires at least two points. Received {}".format(
                len(marker.points)))
      return

    last_point = None
    thickness = marker.scale.x
    for point in marker.points:
      if last_point:
        start = carla.Location(x=last_point.x, y=-last_point.y, z=last_point.z)
        end = carla.Location(x=point.x, y=-point.y, z=point.z)
        logging.info("Draw Line from {} to {} (color: {}, lifetime: {}, "
                                  "thickness: {})".format(start, end, color, lifetime, thickness))
        self.debug.draw_line(start, end, thickness=thickness, color=color,
                                     life_time=lifetime)
      last_point = point

  def draw_box(self, marker, lifetime, color):
    box = carla.BoundingBox()
    box.location.x = marker.pose.position.x
    box.location.y = -marker.pose.position.y
    box.location.z = marker.pose.position.z
    box.extent.x = marker.scale.x / 2
    box.extent.y = marker.scale.y / 2
    box.extent.z = marker.scale.z / 2

    roll, pitch, yaw = quat2euler([
      marker.pose.orientation.w,
      marker.pose.orientation.x,
      marker.pose.orientation.y,
      marker.pose.orientation.z])
    rotation = carla.Rotation()
    rotation.roll = math.degrees(roll)
    rotation.pitch = math.degrees(pitch)
    rotation.yaw = -math.degrees(yaw)
    logging.info("Draw Box {} (rotation: {}, color: {}, lifetime: {})".format(
            box, rotation, color, lifetime))
    self.debug.draw_box(box, rotation, thickness=0.1, color=color, life_time=lifetime)
