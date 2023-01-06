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

from modules.common.proto.header_pb2 import Header


class PseudoActor(object):
  def __init__(self, uid, name, parent, node):
    self.uid = uid
    self.name = name
    self.parent = parent
    self.node = node

    if self.uid is None:
      raise TypeError("Actor ID is not set")

    if self.uid > np.iinfo(np.uint32).max:
      raise ValueError("Actor ID exceeds maximum supported value '{}'".format(self.uid))

  def destroy(self):
    self.parent = None

  @staticmethod
  def get_blueprint_name():
    raise NotImplementedError("The pseudo actor is missing a blueprint name")

  def get_msg_header(self, frame_id=None, timestamp=None):
    header = Header()
    if frame_id:
      header.frame_id = frame_id
    else:
      header.frame_id = self.get_prefix()

    if not timestamp:
      timestamp = self.node.get_time()
    header.timestamp_sec = timestamp
    return header

  def get_prefix(self):
    if self.parent is not None:
      return self.parent.get_prefix() + "/" + self.name
    else:
      return self.name

  def get_topic_prefix(self):
    return "/carla/" + self.get_prefix()

  def update(self, frame, timestamp):
    pass
