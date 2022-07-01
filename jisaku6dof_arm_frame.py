#!/usr/bin/env python
# -*- coding: UTF-8 -*-

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

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
#from cmath import acos, atan
from cv2 import QT_STYLE_OBLIQUE
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import csv
from moveit_msgs.msg import PlanningScene, ObjectColor
import numpy as np
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetStateValidity
from template import User
# from simulation3D import Simulation

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from std_msgs.msg import Header
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import MultiDOFJointState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetStateValidityRequest


## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        # group_name = "jisaku6dof_arm"
        group_name = "jisaku6dof_arm_no_virtual_joint"
        # group_name = "virtual_joint"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used t o display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
    
    def go_to_start_joint_state(self):
        move_group = self.move_group
        #print(move_group.get_interface_description())
        # print("get_named_targets()", move_group.get_named_targets())
        # print("get_named_target_values(target)", move_group.get_named_target_values("start"))
        
        move_group.set_named_target("start")
        move_group.go(wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()

        #print(move_group.get_planner_id())
        print('yeah')
        #print(move_group.get_interface_description())

    def get_fk(self, joint_positions=None):
        robot = self.robot

        compute_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)
        request = GetPositionFKRequest()
        # request.header = Header()
        link_name = robot.get_link_names()
        # print(link_name)
        request.fk_link_names = link_name
        robot_state = robot.get_current_state()
        request.robot_state = robot_state
        if not joint_positions is None:
            request.robot_state.joint_state.position = joint_positions
        response = compute_fk(request)
        eef_pose_stamped = response.pose_stamped[len(response.fk_link_names)-1]
        frame_id = eef_pose_stamped.header.frame_id
        eef_position = eef_pose_stamped.pose.position
        # print("frame_id: {} \neef_position: \n{}" .format(frame_id, eef_position))
        
        return frame_id, eef_position

        # for i in range(len(response.pose_stamped)):
        #     print("{}:\n{}" .format(request.fk_link_names[i], response.pose_stamped[i].pose.position))

    def check_collision(self, joint_positions=None):
        robot = self.robot
        move_group = self.move_group
        
        check_state_validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        request = GetStateValidityRequest()
        robot_state = robot.get_current_state()
        request.robot_state = robot_state
        if not joint_positions is None:
            request.robot_state.joint_state.position = joint_positions
        group_name = move_group.get_name()
        request.group_name = group_name
        response = check_state_validity(request)
        valid = response.valid

        return valid

    def set_start_state(self, rotation=None):
        move_group = self.move_group
        robot = self.robot
        if rotation == None:
            rotation = [0., 0., 0.70710678, 0.70710678]

        # euler = np.array([ 0, 0, 30])
        # rot = Rotation.from_euler('XYZ', euler, degrees=True)
        # rotation = rot.as_quat()
        # rotation = [0., 0., 0.25881905, 0.96592583]
        robot_state = robot.get_current_state()
        multi_dof_joint_state = robot_state.multi_dof_joint_state
        print(multi_dof_joint_state.transforms[0].rotation)
        print("->")
        multi_dof_joint_state.transforms[0].rotation.x = rotation[0]
        multi_dof_joint_state.transforms[0].rotation.y = rotation[1]
        multi_dof_joint_state.transforms[0].rotation.z = rotation[2]
        multi_dof_joint_state.transforms[0].rotation.w = rotation[3]
        print(multi_dof_joint_state.transforms[0].rotation)
        print("")
        # print(robot_state)

        move_group.set_start_state(robot_state)


    def plan_on_error_base(self):
        move_group = self.move_group
        robot = self.robot
        robot_state = robot.get_current_state()

        self.set_start_state()
        #print(move_group.get_interface_description())
        #move_group.set_planner_id('RRTkConfigDefault')
        target_position = [0.3, 0.2, 0.1]
        move_group.set_position_target(target_position)
        # plan = move_group.go(wait=True)
        plan = move_group.plan()[1]
        print(plan)
        self.plan = plan
        # move_group.execute(plan, wait=True)
        # move_group.stop()
        move_group.clear_pose_targets()
        # end_effector_link = move_group.get_end_effector_link()
        # print("end_effector: {}".format(end_effector_link))
        current_pose = self.move_group.get_current_pose().pose
        print("current_pose: {}". format(current_pose))
        print("target_position = {}" .format(target_position))
        
        self.target_position = target_position

    def plan_joints2positions(self):
        plan = self.plan
        target_position = self.target_position

        points = plan.joint_trajectory.points
        num_points = len(points)
        print("経路点: {}個" .format(num_points))
        for i in range(num_points):
            joint_positions = points[i].positions
            frame_id, eef_position = self.get_fk(joint_positions)
            valid_bool = self.check_collision(joint_positions)
            if i == 0:
                print("frame_id: {}" .format(frame_id))
            print("{}: joint_positions = {}" .format(i, joint_positions))
            print("eef_position = {}" .format(eef_position))
            if not valid_bool:
                print("valid" .format(valid_bool))

        print("\noriginal target_position = {}" .format(target_position))

    def execute_plan_on_error_base(self):
        move_group = self.move_group
        robot = self.robot
        plan = self.plan
        target_position = self.target_position

        robot_state = robot.get_current_state()
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        # end_effector_link = move_group.get_end_effector_link()
        # print("end_effector: {}".format(end_effector_link))
        current_pose = self.move_group.get_current_pose().pose
        print("current_pose: {}". format(current_pose))
        print("\noriginal target_position = {}" .format(target_position))

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        print(display_trajectory)
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        target_position = [0.2, -0.3, 0.1]
        box_pose.pose.position.x = target_position[0]
        box_pose.pose.position.y = target_position[1]
        box_pose.pose.position.z = target_position[2]
        box_name = "box"
        height = 0.1
        radius = 0.02
        scene.add_cylinder(box_name, box_pose, height, radius)

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()
        # Simulation3D = Simulation()

        # input("============ Press `Enter` to test pybind11 ...")
        # tutorial.test()

        for i in range(2):

            # input("============ Press `Enter` to test simulation3D ...")
            # Simulation3D.main_loop()
            input("============ Press `Enter` to test move using start joint state ...")
            tutorial.go_to_start_joint_state()

            input("============ Press `Enter` to compute fk ...")
            frame_id, eef_position = tutorial.get_fk()
            print(frame_id)
            print(eef_position)

            input("============ Press `Enter` to plan a movement using a position goal on error base ...")
            tutorial.plan_on_error_base()

            input("============ Press `Enter` to add a box to the planning scene ...")
            tutorial.add_box()

            input("============ Press `Enter` to convert plan from joints to eef position ...")
            tutorial.plan_joints2positions()

            input("============ Press `Enter` to execute plan a movement using a position goal on error base ...")
            tutorial.execute_plan_on_error_base()

            input("============ Press `Enter` to remove the box from the planning scene ...")
            tutorial.remove_box()


        input("============ Press `Enter` to add a box to the planning scene ...")
        tutorial.add_box()

        input("============ Press `Enter` to remove the box from the planning scene ...")
        tutorial.remove_box()

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()


## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/a基準i/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/noetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
