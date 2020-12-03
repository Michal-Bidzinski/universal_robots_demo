#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

import sys
import copy
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from ur_library import all_close, MoveGroupPythonInteface


def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    move_group = MoveGroupPythonInteface()
    rospy.sleep(1)

    print "Go to home"
    move_group.go_to_joint_state(0, -50, 80, -30, 90, 0)

    print "============ Press `Enter` to plan and execute movements..."
    raw_input()

    print "*** Plan and execute first point ***"
    plan1 = move_group.plan_path_to_goal(0.134, 0.328, 0.038, 0, 0, 0, 1)
    move_group.execute_plan(plan1)
    print "*** Plan and execute second point ***"
    plan2 = move_group.plan_path_to_goal(0.3, 0.0, 0.2, 0, 0, 0, 1)
    move_group.execute_plan(plan2)

    #print "============ Press `Enter` to execute a movement using a pose goal ..."
    #raw_input()
    print "*** Plan and execute sequence of two other points ***"
    waypoints_array = []
    waypoints_array.append([0.3, 0.26496604472, 0.320554352366, 0, 0, 0, 1, False])
    waypoints_array.append([0.3, -0.26496604472, 0.320554352366, 0, 0, 0, 1, False])
    plan3 = move_group.create_cartesian_path(waypoints_array)
    move_group.execute_plan(plan3)

    print "Go to home"
    move_group.go_to_joint_state(0, -50, 80, -30, 90, 0)

    print "============ Press `Enter` to execute all movements again ..."
    raw_input()

    move_group.go_to_joint_state(0, -50, 80, -30, 90, 0)

    move_group.execute_plan(plan1)
    move_group.execute_plan(plan2)
    move_group.execute_plan(plan3)



    #print "============ Press `Enter` to go to home =============="
    #raw_input()
    #move_group.go_to_joint_state(0, -50, 80, -30, 90, 0)




    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

