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
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import csv
from moveit_msgs.msg import PlanningScene, ObjectColor

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

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

        scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.scene_pub = scene_pub
        self.colors = dict()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "shifted_jisaku6dof_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
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

    def add_box(self, frame, timeout=4):
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

        if frame == "base":
            box_pose.header.frame_id = "base"
            box_name1 = "box1"
            box_name2 = "box2"
            box_name3 = 'box3'
            error_x = 0
            error_y = 0
            print("yeah")
        if frame == "shifted_base":
            box_pose.header.frame_id = "shifted_base"
            box_name1 = "shifted_box1"
            box_name2 = "shifted_box2"
            box_name3 = "shifted_box3"
            error_x = 0.002
            error_y = 0.005
        # boxの中心の座標を入力
        # z座標0: xy平面がboxの中心になる    
        box_pose.pose.position.x += 0.3 + error_x
        box_pose.pose.position.y += -0.2 - error_y
        box_pose.pose.position.z += 0.25
        box_pose.pose.orientation.w = 1.0
        print(box_pose.pose.position)
        print("\n")
        self.setColor(box_name1)
        self.sendColors()
        scene.add_box(box_name1, box_pose, size=(0.02, 0.02, 0.5))



        box_pose.pose.position.x += -0.2 #+ error_x
        box_pose.pose.position.y += 0 #- error_y
        box_pose.pose.position.z += 0
        box_pose.pose.orientation.w = 1.0
        print(box_pose.pose.position)
        self.setColor(box_name2)
        self.sendColors()
        scene.add_box(box_name2, box_pose, size=(0.02, 0.02, 0.5))

        box_pose.pose.position.x += 0 #+ error_x
        box_pose.pose.position.y = 0 + 0.01 - error_y
        box_pose.pose.position.z += 0 + 0.25 -0.01
        box_pose.pose.orientation.w = 1.0
        print(box_pose.pose.position)
        self.setColor(box_name3)
        self.sendColors()
        scene.add_box(box_name3, box_pose, size=(0.02, 0.4, 0.02))

        
        

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def setColor(self, name, r=0.8, g=0.8, b=0.8, a = 0.4):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color
    
    def sendColors(self):
        p = PlanningScene()
        p.is_diff = True
        for color in self.colors.values():
            p.object_colors.append(color)
        self.scene_pub.publish(p)

    def shifted_go_to_joint_start_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:

        #print(move_group.get_interface_description())
        #print(move_group.get_planner_id())
        #move_group.set_planner_id('shifted_jisaku6dof_arm[RRTConnect]')
        move_group.set_planner_id('shifted_jisaku6dof_arm[TRRT]')
        #print(move_group.get_planner_id())
        
        
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = pi/4 #-tau / 9
        joint_goal[2] = -3*pi/4 #-tau / 9
        joint_goal[3] = 0 #-tau / 6
        joint_goal[4] = -pi/2
        joint_goal[5] = 0 #tau / 6  # 1/6 of a turn
        #joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        current_pose = move_group.get_current_pose()
        print(current_joints)
        print(current_pose)


        #print('yeah')

        return all_close(joint_goal, current_joints, 0.01)


    def shifted_check_go_to_plan(self):
        move_group = self.move_group
        scene = self.scene


        object_position = scene.get_object_poses(["shifted_box1"])
        object = scene.get_objects(["shifted_box1"])
        #print(position)
        #print(object)
        object_pos_x = object_position.get('shifted_box1').position.x
        object_pos_y = object_position.get('shifted_box1').position.y
        #print(object_pos_x)

        object_demensions_x = object.get('shifted_box1').primitives[0].dimensions[0]
        object_demensions_y = object.get('shifted_box1').primitives[0].dimensions[0]
        #print(object_demensions_x)
        goal_pos = geometry_msgs.msg.Pose().position

        goal_pos.x = object_pos_x + object_demensions_x/2 + 0.02
        goal_pos.y = object_pos_y + object_demensions_y/2 - 0.02
        goal_pos.z = 0.4
        print(goal_pos)
        
        #print(move_group.get_interface_description())
        print(move_group.get_planner_id())
        move_group.set_position_target([
            goal_pos.x, goal_pos.y, goal_pos.z
            ])
        
        plan = move_group.plan()
        move_group.execute(plan[1], wait=True)

        points = plan[1].joint_trajectory.points
        num_points = len(points)
        print(num_points)

        current_pose = self.move_group.get_current_pose().pose
        print(current_pose)

        self.plan = plan
        return all_close(goal_pos, current_pose, 0.01)


    def shifted_go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.5
        pose_goal.position.y = 0.3
        pose_goal.position.z = -0.5


        print(move_group.get_interface_description())

        #move_group.set_planner_id('RRTkConfigDefault')

        #move_group.set_pose_target(pose_goal)

        move_group.set_position_target([
            pose_goal.position.x, pose_goal.position.y, pose_goal.position.z])

        #move_group.set_orientation_target([0, -1, 0, 0])

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.plan()
        
        # plan = (True, moveit_msgs/RobotTrajectory)
        # 欲しいのは plan[1]
        
        move_group.execute(plan[1], wait=True)

        #print(plan)

        #print(move_group.get_pose_reference_frame())
        print(move_group.get_planning_frame())

        #with open('/home/cshiba/kyoudoukenkyu/autonomous-control-simulation-master/plan.csv', 'w') as f:
        #    writer = csv.writer(f)
        #    writer.writerows(plan)


        #move_group.go(wait=True)
        #move_group.execute(plan, wait=True)
        
        # Calling `stop()` ensures that there is no residual movement
        #move_group.stop()
        
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        #move_group.clear_pose_targets()

        points = plan[1].joint_trajectory.points

        num_points = len(points)

        print(num_points)

        joint_goal = move_group.get_current_joint_values()

        joints_goal = copy.deepcopy(joint_goal)

        num_joint = len(joints_goal)
        print(num_joint)

        trajectory_position = []

        for i in range(num_points):
            trajectory_joint = points[i].positions

            for j in range(num_joint):
                joints_goal[j] = trajectory_joint[j]
                #print(joints_goal[j])
            #print(i, ":",  joints_goal)
            #move_group.go(joints_goal, wait=True)

            x = move_group.get_current_pose().pose.position.x
            y = move_group.get_current_pose().pose.position.y
            z = move_group.get_current_pose().pose.position.z

            trajectory_position.append([x, y,z])
            #print(trajectory_position[i])

        #with open('/home/cshiba/kyoudoukenkyu/autonomous-control-simulation-master/trajectory_position_list.csv', 'w') as f:
        #    writer = csv.writer(f)
        #    writer.writerows(trajectory_position)

        

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = move_group.get_current_pose().pose
        print(current_pose)

        self.plan = plan
        return all_close(pose_goal, current_pose, 0.01)
    

    def update_setting(self):

        print("update setting from jisaku6def_arm to shifted_jisaku6dof_arm")

        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot2 = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene2 = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        
        group_name2 = "jisaku6def_arm"
        move_group2 = moveit_commander.MoveGroupCommander(group_name2)

        # We can get the name of the reference frame for this robot:
        planning_frame2 = move_group2.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame2)

        # We can also print the name of the end-effector link for this group:
        eef_link2 = move_group2.get_end_effector_link()
        print("============ End effector link: %s" % eef_link2)

        # We can get a list of all the groups in the robot:
        group_names2 = robot2.get_group_names()
        print("============ Available Planning Groups:", robot2.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot2.get_current_state())
        print("")

        self.robot2 = robot2
        self.scene2 = scene2
        self.move_group2 = move_group2
        self.planning_frame2 = planning_frame2
        self.eef_link2 = eef_link2
        self.group_names2 = group_names2

    def go_to_joint_start_state(self):
        move_group2 = self.move_group2

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:

        #print(move_group2.get_interface_description())
        #print(move_group2.get_planner_id())
        move_group2.set_planner_id('jisaku6def_arm[RRTConnect]')
        #print(move_group2.get_planner_id())

        joint_goal = move_group2.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = pi/4 #-tau / 9
        joint_goal[2] = -3*pi/4 #-tau / 9
        joint_goal[3] = 0 #-tau / 6
        joint_goal[4] = -pi/2
        joint_goal[5] = 0 #tau / 6  # 1/6 of a turn
        #joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group2.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group2.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group2.get_current_joint_values()
        current_pose = move_group2.get_current_pose()
        print(current_joints)
        print(current_pose)

        #print(move_group.get_planner_id())
        print('yeah')
        #print(move_group.get_interface_description())



        return all_close(joint_goal, current_joints, 0.01)




    def go_to_plan(self):
        plan = self.plan
        move_group2 = self.move_group2

        print(move_group2.get_planner_id())

        #print(plan[1])

        names = plan[1].joint_trajectory.joint_names
        print(names)
        names[0] = 'joint1'
        names[1] = 'joint2'
        names[2] = 'joint3'
        names[3] = 'joint4'
        names[4] = 'joint5'
        names[5] = 'joint6'

        print(names)

        

        move_group2.execute(plan[1], wait=True)
        print(move_group2.get_current_pose())


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
        tutorial = MoveGroupPythonInterfaceTutorial()


        input("============ Press `Enter` to add a box to the base planning scene ...")
        tutorial.add_box("base")

        input("============ Press `Enter` to add a box to the shifted base planning scene ...")
        tutorial.add_box("shifted_base")



        input("============ Press `Enter` to execute a movement using a joint start state by shifted_arm ...")
        tutorial.shifted_go_to_joint_start_state()

        input("============ Press `Enter` to execute a movement using a pose goal by shifted_arm ...")
        #tutorial.go_to_pose_goal()
        tutorial.shifted_check_go_to_plan()



        input("============ Press `Enter` to update setting from shifted_jisaku6dof_arm to jisaku6def_arm ...")
        tutorial.update_setting()



        input("============ Press `Enter` to execute a movement using a joint start state ...")
        tutorial.go_to_joint_start_state()

        input("============ Press `Enter` to execute a movement using a plan by jisaku6dof_arm ...")
        tutorial.go_to_plan()





        

        #input("============ Press `Enter` to attach a Box to the Panda robot ...")
        #tutorial.attach_box()

        #input(
        #    "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        #)
        #cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        #tutorial.execute_plan(cartesian_plan)

        #input("============ Press `Enter` to detach the box from the Panda robot ...")
        #tutorial.detach_box()

        #input(
        #    "============ Press `Enter` to remove the box from the planning scene ..."
        #)
        #tutorial.remove_box()

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
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
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
