#!/usr/bin/env python


import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


def callback(message):

    pos = message
    rospy.loginfo("get /move_group/display_planned_path \n\n%s\n\n\n\n", pos)

def callback2(message):

    
    rospy.loginfo("get /move_group/display_planned_path \n\n%s\n\n\n\n", message)


rospy.init_node('display_planned_path_subscriber')

sub = rospy.Subscriber("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, callback)
#sub = rospy.Subscriber("/move_group/", moveit_msgs.msg.RobotTrajectory, callback2)

rospy.spin()