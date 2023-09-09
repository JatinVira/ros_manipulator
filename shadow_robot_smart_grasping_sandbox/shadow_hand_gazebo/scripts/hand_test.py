#!/usr/bin/env python

import rospy
from smart_grasping_sandbox.smart_grasper import SmartGrasper
from tf.transformations import quaternion_from_euler
from math import pi
import time


def open_close_test():

    sgs = SmartGrasper()

    sgs.pick()

    sgs.reset_world()

    return None


if __name__ == "__main__":
    open_close_test()
