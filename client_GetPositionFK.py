#!/usr/bin/env python



import rospy
import moveit_msgs.srv

def add_two_ints_client():
    rospy.wait_for_service('/compute_fk')
    try:
        add_two_ints = rospy.ServiceProxy('/compute_fk', moveit_msgs.srv.GetPositionFK)
        resp1 = add_two_ints
        print("ta")
        return resp1
    except rospy.ServiceException:
        print ("Service call failed: ")



if __name__ == "__main__":
    print(add_two_ints_client)

    print("saa")