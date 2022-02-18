#!/usr/bin/env python

import IPython

import rospy

from base_task_interface import BaseTaskInterface
from reorientbot_task_interface import ReorientbotTaskInterface
from safepicking_task_interface import SafepickingTaskInterface


class CombinedTaskInterface:
    def __init__(self, base):
        self.base = base
        self.safepicking = SafepickingTaskInterface(self.base)
        self.reorientbot = ReorientbotTaskInterface(self.base)

    def run(self):
        self.safepicking.run(self, place=False)
        self.reorientbot.run(self)


rospy.init_node("combined_task_interface")
base = BaseTaskInterface
self = CombinedTaskInterface(base)
IPython.embed()
