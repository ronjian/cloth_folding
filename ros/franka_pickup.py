#!/usr/bin/env python3
import rospy
import time, math
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander
from control_msgs.msg import GripperCommandActionGoal
from threading import Thread


class Manipulator:
    def __init__(self, arm_name):
        self.arm_group = MoveGroupCommander(name="panda_manipulator", ns=arm_name)
        self.arm_group.set_pose_reference_frame("panda_link0")
        self.arm_group.set_end_effector_link("panda_hand_tcp")
        self.gripper_group = MoveGroupCommander("panda_hand", ns=arm_name)
        self.gripper_pub = rospy.Publisher(
            "/{}/franka_gripper/gripper_action/goal".format(arm_name),
            GripperCommandActionGoal,
            queue_size=10,
        )
        self.arm_name = arm_name

    def move_to_joints(self, joints):
        assert len(joints) == 7, "关节数不正确！"
        self.arm_group.set_joint_value_target(
            [p * math.pi / 180.0 for p in joints]
        )
        # 执行运动
        self.arm_group.go(wait=True)
        return

    def move_to_start(self):
        self.move_to_joints([0, -45, 0, -135, 0, 90, 45])
        rospy.loginfo("%s | 运动到起始位置完成！" % self.arm_name)

    def move_to_home(self):
        self.move_to_joints([0, -45, 0, -45, 0, 90, 45])
        rospy.loginfo("%s | 运动到HOME位置完成！" % self.arm_name)

    def move_gripper(self, position):
        self.gripper_group.set_joint_value_target(
            [position, position]
        )  # Panda夹爪有两个对称关节

        # 发送夹爪开口大小给isaac sim
        goal_msg = GripperCommandActionGoal()
        goal_msg.goal.command.position = position  # 开口大小（米，0.0~0.04）
        self.gripper_pub.publish(goal_msg)

        # 执行运动
        self.gripper_group.go(wait=True)

    def open_gripper(self):
        self.move_gripper(0.04)
        rospy.loginfo("%s | 夹爪打开完成！" % self.arm_name)

    def close_gripper(self):
        self.move_gripper(0.0)
        rospy.loginfo("%s | 夹爪闭合完成！" % self.arm_name)

    def move_straight(self, target_pose: Pose):
        current_pose = self.arm_group.get_current_pose().pose
        waypoints = [current_pose, target_pose]
        tic = time.time()
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints, 0.01, avoid_collisions=True  # 路径点列表  # 步长（米）
        )
        rospy.loginfo("%s | 规划时间：%.2f秒" % (self.arm_name, time.time() - tic))
        # 执行轨迹
        if fraction < 1.0:
            rospy.loginfo(
                "%s | 规划失败，仅完成%.1f%%路径, 直接运动到目标位置" % (self.arm_name, fraction * 100)
            )
            # self.arm_group.set_pose_target(target_pose)
            # rc = self.arm_group.go(wait=True)
            # print(rc)
            # cur_orientation_tolerance = self.arm_group.get_goal_orientation_tolerance()
            # self.arm_group.get_goal_tolerance()
            # if cur_orientation_tolerance > math.pi:
            #     return -1
            # self.arm_group.set_goal_orientation_tolerance(
            #     cur_orientation_tolerance + 0.5
            # )
            # self.move_straight(target_pose)
        else:
            self.arm_group.execute(plan, wait=True)
            rospy.loginfo("%s | 直线运动完成！" % self.arm_name)
        return 0

    def current_pose(self):
        return self.arm_group.get_current_pose().pose

    def pickup(self):
        self.move_to_home()
        self.move_to_start()
        # 获取当前末端姿态
        start_pose = self.current_pose()
        # 定义目标点
        target_pose = Pose()
        target_pose.position.x = start_pose.position.x
        target_pose.position.y = start_pose.position.y
        target_pose.position.z = 0.01
        target_pose.orientation = start_pose.orientation

        self.move_straight(target_pose)
        self.close_gripper()
        self.move_straight(start_pose)
        # self.move_to_start()
        self.open_gripper()
        self.move_to_home()
    
    def pick_and_place(self, pick_point, place_point):
        self.move_to_start()
        # pick
        start_pose = self.current_pose()
        pick_pose = Pose()
        pick_pose.position.x = pick_point[0]
        pick_pose.position.y = pick_point[1]
        pick_pose.position.z = pick_point[2] - 0.015
        pick_pose.orientation = start_pose.orientation
        self.move_straight(pick_pose)
        self.close_gripper()
        # up
        pick_pose.position.z = 0.2
        self.move_straight(pick_pose)
        # transfer
        place_pose = Pose()
        place_pose.position.x = place_point[0]
        place_pose.position.y = place_point[1]
        place_pose.position.z = 0.2
        place_pose.orientation = start_pose.orientation
        self.move_straight(place_pose)
        # place
        place_pose.position.z = place_point[2] + 0.01
        self.move_straight(place_pose)
        self.open_gripper()
        # drawback
        place_pose.position.z = place_point[2] + 0.02
        self.move_straight(place_pose)
        self.move_to_start()
        self.move_to_home()

if __name__ == "__main__":
    rospy.init_node("franka_cartesian_move")
    panda_left = Manipulator("panda_left")
    panda_right = Manipulator("panda_right")
    Thread(target=panda_right.pickup).start()
    Thread(target=panda_left.pickup).start()
