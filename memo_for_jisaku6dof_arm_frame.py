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