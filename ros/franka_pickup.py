#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander
from control_msgs.msg import GripperCommandActionGoal
from threading import Thread
import copy
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading

# 创建屏障，设置需要等待的线程数（这里是2）
barrier = threading.Barrier(2)

def scale_plan_speed(plan, speed_scale=1.0):
    # 修改轨迹时间参数
    new_trajectory = JointTrajectory()
    new_trajectory.joint_names = plan.joint_trajectory.joint_names

    time_scaling = 1.0 / speed_scale

    # 重新计算各点时间
    for i, point in enumerate(plan.joint_trajectory.points):
        new_point = JointTrajectoryPoint()
        new_point.positions = point.positions
        new_point.velocities = point.velocities
        new_point.accelerations = point.accelerations
        new_point.effort = point.effort
        new_point.time_from_start = rospy.Duration(time_scaling * point.time_from_start.to_sec())
        # rospy.loginfo("time_from_start: from %s to %s" % (point.time_from_start, new_point.time_from_start))
        new_trajectory.points.append(new_point)

    # 使用修改后的轨迹
    plan.joint_trajectory = new_trajectory
    return plan

class Manipulator:
    HOME_POSITION = [p * math.pi / 180.0 for p in [0, -45, 0, -45, 0, 90, 45]]
    START_POSITION = [p * math.pi / 180.0 for p in [0, -45, 0, -135, 0, 90, 45]]
    EXTENSION_POSITION = [p * math.pi / 180.0 for p in [0, -45, 0, -135, 0, 145, 45]]
    def __init__(self, arm_name):
        self.arm_group = MoveGroupCommander(name="panda_manipulator"
                                            , robot_description="/{}/robot_description".format(arm_name)
                                            , ns=arm_name)
        self.arm_group.set_pose_reference_frame("panda_link0")
        self.arm_group.set_end_effector_link("panda_hand_tcp")
        self.gripper_group = MoveGroupCommander("panda_hand"
                                                , robot_description="/{}/robot_description".format(arm_name)
                                                , ns=arm_name)
        self.gripper_pub = rospy.Publisher(
            "/{}/franka_gripper/gripper_action/goal".format(arm_name),
            GripperCommandActionGoal,
            queue_size=10,
        )
        self.arm_name = arm_name
        self.tf_listener = tf.TransformListener()

    def move_to_joints(self, joints):
        assert len(joints) == 7, "关节数不正确！"
        self.arm_group.set_joint_value_target(joints)
        self.arm_group.go(wait=True)
        return

    def move_to_start(self):
        self.move_to_joints(self.START_POSITION)
        rospy.loginfo("%s | 运动到起始位置完成！" % self.arm_name)

    def move_to_extension(self):
        self.move_to_joints(self.EXTENSION_POSITION)
        rospy.loginfo("%s | 运动到扩展位置完成！" % self.arm_name)

    def move_to_home(self):
        self.move_to_joints(self.HOME_POSITION)
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

    def move_straight(self, target_pose: Pose, speed_scale=1.0):
        current_pose = self.current_pose
        waypoints = [current_pose, target_pose]
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints, 0.01, avoid_collisions=True  # 路径点列表  # 步长（米）
        )
        # 执行轨迹
        if fraction < 1.0:
            rospy.loginfo(
                "%s | 规划失败，仅完成%.1f%%路径" % (self.arm_name, fraction * 100)
            )
            # self.arm_group.set_pose_target(target_pose)
            # self.arm_group.go(wait=True)
            return -1
        if speed_scale != 1.0:
            plan = scale_plan_speed(plan, speed_scale)
        self.arm_group.execute(plan, wait=True)
        rospy.loginfo("%s | 直线运动完成！" % self.arm_name)
        return 0

    @property
    def current_pose(self):
        # return self.arm_group.get_current_pose("panda_hand_tcp").pose
        # TODO: 这里需要使用tf来获取当前末端位姿, 因为moveit的当前位姿获取有问题， 好像是robot_description配置不对
        base_link = "/{}/panda_link0".format(self.arm_name)
        end_effector = "/{}/panda_hand_tcp".format(self.arm_name)
        self.tf_listener.waitForTransform(base_link, end_effector, rospy.Time(), rospy.Duration(4.0))
        # 获取平移和旋转
        (xyz, xyzw) = self.tf_listener.lookupTransform(base_link, end_effector, rospy.Time(0))
        cur_pose = Pose()
        cur_pose.position.x = xyz[0]
        cur_pose.position.y = xyz[1]
        cur_pose.position.z = xyz[2]
        cur_pose.orientation.x = xyzw[0]
        cur_pose.orientation.y = xyzw[1]
        cur_pose.orientation.z = xyzw[2]
        cur_pose.orientation.w = xyzw[3]
        return cur_pose
    
    def current_joints(self):
        cur_joints = copy.copy(self.arm_group.get_current_joint_values())
        return cur_joints

    def pickup(self):
        self.move_to_home()
        self.move_to_start()
        # 获取当前末端姿态
        start_pose = self.current_pose
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
    
    def pick_and_place(self, pick_point, place_point, sync = False, deep = False):
        picking_height = 0.3
        rospy.loginfo(f"{self.arm_name} | 捡起点：{pick_point}，放下点：{place_point}")
        rospy.loginfo("%s | 运动到起始位置" % self.arm_name)
        self.move_to_start()
        self.open_gripper()
        if sync:
            barrier.wait()
        rospy.loginfo("%s | 运动到抓取位置" % self.arm_name)
        current_joints = copy.copy(self.START_POSITION)
        current_joints[0] += math.atan2(pick_point[1], pick_point[0])
        self.move_to_joints(current_joints) 
        current_pose = self.current_pose
        pick_pose = Pose()
        pick_pose.position.x = pick_point[0]
        pick_pose.position.y = pick_point[1]
        if deep:
            z_height = max(pick_point[2] - 0.04, 0.01) 
        else:
            z_height = max(pick_point[2] - 0.02, 0.01)
        rospy.loginfo("%s | 运动到抓取高度 %s" % (self.arm_name, z_height))
        pick_pose.position.z = z_height
        pick_pose.orientation = current_pose.orientation
        rc = self.move_straight(pick_pose)
        if rc < 0:
            return -1
        rospy.loginfo("%s | 关闭夹爪" % self.arm_name)
        self.close_gripper()
        if sync:
            barrier.wait()
        rospy.loginfo("%s | 运动到提起位置" % self.arm_name)
        pick_pose.position.z = picking_height
        rc = self.move_straight(pick_pose)
        if rc < 0:
            return -1
        if sync:
            barrier.wait()
        rospy.loginfo("%s | 运动到转移位置" % self.arm_name)
        current_pose = self.current_pose
        place_pose = Pose()
        place_pose.position.x = place_point[0]
        place_pose.position.y = place_point[1]
        place_pose.position.z = picking_height
        place_pose.orientation = current_pose.orientation
        rc = self.move_straight(place_pose, speed_scale=0.2)
        if rc < 0:
            return -1
        if sync:
            barrier.wait()
        rospy.loginfo("%s | 运动到放置位置" % self.arm_name)
        # place
        place_pose.position.z = place_point[2] + 0.05
        rc = self.move_straight(place_pose)
        if rc < 0:
            return -1
        if sync:
            barrier.wait()
        rospy.loginfo("%s | 打开夹爪" % self.arm_name)
        self.open_gripper()
        rospy.loginfo("%s | 收回机械臂" % self.arm_name)
        self.move_to_start()
        self.move_to_home()

    def flatten(self, pick_point, sync = False):
        self.open_gripper()
        self.move_to_extension()
        extension_orientation = self.current_pose.orientation
        picking_height = 0.5
        rospy.loginfo(f"{self.arm_name} | 捡起点：{pick_point}")
        rospy.loginfo("%s | 运动到起始位置" % self.arm_name)
        # pick
        rospy.loginfo("%s | 运动到抓取位置" % self.arm_name)
        current_joints = copy.copy(self.START_POSITION)
        current_joints[0] += math.atan2(pick_point[1], pick_point[0])
        self.move_to_joints(current_joints) 
        current_pose = self.current_pose
        pick_pose = Pose()
        pick_pose.position.x = pick_point[0]
        pick_pose.position.y = pick_point[1]
        z_height = max(pick_point[2] - 0.02, 0.01)
        rospy.loginfo("%s | 运动到抓取高度 %s" % (self.arm_name, z_height))
        pick_pose.position.z = z_height
        pick_pose.orientation = current_pose.orientation
        rc = self.move_straight(pick_pose)
        if rc < 0:
            return -1
        rospy.loginfo("%s | 关闭夹爪" % self.arm_name)
        self.close_gripper()
        if sync:
            barrier.wait()
        # up
        rospy.loginfo("%s | 运动到提起位置" % self.arm_name)
        pick_pose.position.z = picking_height
        rc = self.move_straight(pick_pose, speed_scale=0.2)
        if rc < 0:
            return -1
        if sync:
            barrier.wait()
        rospy.loginfo("%s | 运动到铺平起始位置" % self.arm_name)
        flatten_height = 0.65
        flatten_y = 0.55
        flatten_x = 0.55
        start_pose = Pose()
        start_pose.position.x = flatten_x
        if self.arm_name == 'panda_left':
            start_pose.position.y = -flatten_y
        else:
            start_pose.position.y = flatten_y
        start_pose.position.z = flatten_height
        start_pose.orientation = extension_orientation
        rc = self.move_straight(start_pose, 0.2)
        if rc < 0:
            return -1
        if sync:
            barrier.wait()
        rospy.loginfo("%s | 运动到铺平结束位置" % self.arm_name)
        end_pose = Pose()
        end_pose.position.x = flatten_x
        if self.arm_name == 'panda_left':
            end_pose.position.y = flatten_y
        else:
            end_pose.position.y = -flatten_y
        end_pose.position.z = 0.02
        end_pose.orientation = current_pose.orientation
        rc = self.move_straight(end_pose, 0.2)
        if rc < 0:
            return -1
        if sync:
            barrier.wait()
        rospy.loginfo("%s | 打开夹爪" % self.arm_name)
        self.open_gripper()
        rospy.loginfo("%s | 抬起机械臂" % self.arm_name)
        end_pose.position.z = 0.2
        rc = self.move_straight(end_pose, 0.2)
        if rc < 0:
            return -1
        rospy.loginfo("%s | 收回机械臂" % self.arm_name)
        self.move_to_start()
        self.move_to_home()

if __name__ == "__main__":
    rospy.init_node("franka_cartesian_move")
    panda_left = Manipulator("panda_left")
    panda_right = Manipulator("panda_right")
    Thread(target=panda_right.pickup).start()
    Thread(target=panda_left.pickup).start()
