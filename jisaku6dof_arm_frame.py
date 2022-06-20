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

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import MultiDOFJointState
from moveit_msgs.msg import RobotState

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
    
    def go_to_joint_state(self):
        move_group = self.move_group
        #print(move_group.get_interface_description())
        
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 9
        joint_goal[2] = -tau / 9
        joint_goal[3] = -tau / 6
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()

        #print(move_group.get_planner_id())
        print('yeah')
        #print(move_group.get_interface_description())

        return all_close(joint_goal, current_joints, 0.01)
    
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

    def move_frame_on_multi(self):
        move_group = self.move_group
        robot = self.robot
        robot_state = robot.get_current_state()

        # euler = np.array([ 0, 0, 30])
        # rot = Rotation.from_euler('XYZ', euler, degrees=True)
        # rotation = rot.as_quat()
        rotation = [0., 0., 0.25881905, 0.96592583]
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
        self.robot_state = robot_state

        move_group.set_start_state(robot_state)
        # current_joints = move_group.get_current_joint_values()
        # print(current_joints)
        # print("============ Printing robot state")
        # print(self.robot.get_current_state())
    
    def move_frame_on_go_to_joint(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        print(joint_goal)
        # joint_goal[0] = 0.5
        rotation = [0., 0., 0.70710678, 0.70710678]
        for i in range(4):
            joint_goal[i+3] = rotation[i]
        print(joint_goal)
        move_group.set_joint_value_target("virtual_joint", [0, 0, 0, 0., 0., 0.25881905, 0.96592583])
        # move_group.go(joint_goal, wait=True)
        move_group.go()

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        print(current_joints)
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")


        return all_close(joint_goal, current_joints, 0.01)

    def virtual_joint2arm(self):
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        # group_name = "jisaku6dof_arm"
        group_name = "jisaku6dof_arm_no_virtual_joint"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def set_start_state(self, rotation=None):
        move_group = self.move_group
        robot = self.robot
        robot_state = robot.get_current_state()

        rotation = [0., 0., 0.70710678, 0.70710678]
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


    def go_to_pose_goal(self):
        move_group = self.move_group
        robot = self.robot
        robot_state = robot.get_current_state()

        # euler = np.array([ 0, 0, 30])
        # rot = Rotation.from_euler('XYZ', euler, degrees=True)
        # rotation = rot.as_quat()
        # rotation = [0., 0., 0.25881905, 0.96592583]
        rotation = [0., 0., 0.70710678, 0.70710678]
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

        #print(move_group.get_interface_description())

        #move_group.set_planner_id('RRTkConfigDefault')

        #move_group.set_pose_target(pose_goal)

        move_group.set_position_target([0.3, 0.2, 0.1])

        #move_group.set_orientation_target([0, -1, 0, 0])

        ## Now, we call the planner to compute the plan and execute it.
        # plan = move_group.go(wait=True)
        plan = move_group.plan()[1]
        print(plan)
        self.plan = plan
        # move_group.execute(plan, wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        end_effector_link = move_group.get_end_effector_link()
        print("end_effector: {}".format(end_effector_link))
        current_pose = self.move_group.get_current_pose().pose
        print("current_pose: {}". format(current_pose))
        # return all_close(pose_goal, current_pose, 0.01)

    def check_plan(self):
        move_group = self.move_group
        robot = self.robot
        plan = self.plan
        
        robot_state = robot.get_current_state()
        rotation = [0., 0., 0., 0.]
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

        move_group.execute(plan, wait=True)
        move_group.stop()

        end_effector_link = move_group.get_end_effector_link()
        print("end_effector: {}".format(end_effector_link))
        current_pose = self.move_group.get_current_pose().pose
        print("current_pose: {}". format(current_pose))

    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        #print(plan)

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

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

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
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
        box_pose.header.frame_id = "jisaku6dof_arm_no_virtual_joint"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

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

        # input(
        #     "============ Press `Enter` to test move using joint state ..."
        # )
        # tutorial.go_to_joint_state()
        # input(
        #     "============ Press `Enter` to test move using start joint state ..."
        # )
        # tutorial.go_to_start_joint_state()

        input(
            "============ Press `Enter` to test move frame ..."
        )
        tutorial.move_frame_on_multi()
        # tutorial.move_frame_on_go_to_joint()
        # input(
            # "============ Press `Enter` to shift group virtual_joint2arm ..."
        # )
        # tutorial.virtual_joint2arm()

        


        input("============ Press `Enter` to plan a movement using a position goal ...")
        tutorial.go_to_pose_goal()

        input("============ Press `Enter` to plan and display a Cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        input(
            "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        )
        tutorial.display_trajectory(cartesian_plan)

        input("============ Press `Enter` to test a plan ...")
        tutorial.check_plan()

        input("============ Press `Enter` to plan and display a Cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        input(
            "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        )
        tutorial.display_trajectory(cartesian_plan)

        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)



        input("============ Press `Enter` to attach a Box to the Panda robot ...")
        tutorial.attach_box()

        input(
            "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        )
        cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        tutorial.execute_plan(cartesian_plan)

        input("============ Press `Enter` to detach the box from the Panda robot ...")
        tutorial.detach_box()

        input(
            "============ Press `Enter` to remove the box from the planning scene ..."
        )
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
