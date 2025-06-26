#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander
from control_msgs.msg import GripperCommandActionGoal
import copy
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory

# 创建屏障，设置需要等待的线程数（这里是2）
barrier = threading.Barrier(2)

def scale_plan_speed(plan: RobotTrajectory, speed_scale=1.0):
    # 修改轨迹时间参数
    new_trajectory = JointTrajectory()
    new_trajectory.joint_names = plan.joint_trajectory.joint_names

    time_scaling = 1.0 / speed_scale

    # 重新计算各点时间
    for point in plan.joint_trajectory.points:
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

def position_euler_to_pose(x,y,z,roll, pitch, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    target_quat = quaternion_from_euler(roll, pitch, yaw)
    pose.orientation.x = target_quat[0]
    pose.orientation.y = target_quat[1]
    pose.orientation.z = target_quat[2]
    pose.orientation.w = target_quat[3]
    return pose

class Manipulator:
    HOME_POSITION = [p * math.pi / 180.0 for p in [0, -45, 0, -45, 0, 90, 45]]
    START_POSITION = [p * math.pi / 180.0 for p in [0, -45, 0, -135, 0, 90, 45]]
    VR_POSITION = [p * math.pi / 180.0 for p in [0, -45, 0, -135, 0, 180, 45]]
    JOINTS_NAME = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]

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
        self.joint_states_pub = rospy.Publisher('/{}/move_group/fake_controller_joint_states'.format(arm_name), JointState, queue_size=10)

    def tune_target_pose(self, target_pose, extend=True):
        current_pos = target_pose.position
        current_quat = target_pose.orientation
        roll, pitch, yaw = euler_from_quaternion([current_quat.x, current_quat.y, current_quat.z, current_quat.w])
        if extend:
            pitch -= math.pi / 3
        else:
            pitch += math.pi / 3
        xyzw = quaternion_from_euler(roll, pitch, yaw)
        new_pose = Pose()
        new_pose.position.x = current_pos.x
        new_pose.position.y = current_pos.y
        new_pose.position.z = current_pos.z
        new_pose.orientation.x = xyzw[0]
        new_pose.orientation.y = xyzw[1]
        new_pose.orientation.z = xyzw[2]
        new_pose.orientation.w = xyzw[3]
        return new_pose

    def move_to_joints(self, joints):
        assert len(joints) == 7, "关节数不正确！"
        self.arm_group.set_joint_value_target(joints)
        self.arm_group.go(wait=True)
        return

    def move_to_start(self):
        self.move_to_joints(self.START_POSITION)
        rospy.loginfo("%s | 运动到起始位置完成！" % self.arm_name)

    def move_to_home(self):
        self.move_to_joints(self.HOME_POSITION)
        rospy.loginfo("%s | 运动到HOME位置完成！" % self.arm_name)

    def move_to_vr(self):
        self.move_to_joints(self.VR_POSITION)
        rospy.loginfo("%s | 运动到vr位置完成！" % self.arm_name)

    def move_gripper(self, position, wait=True):
        self.gripper_group.set_joint_value_target(
            [position, position]
        )  # Panda夹爪有两个对称关节

        # 发送夹爪开口大小给isaac sim
        goal_msg = GripperCommandActionGoal()
        goal_msg.goal.command.position = position  # 开口大小（米，0.0~0.04）
        self.gripper_pub.publish(goal_msg)

        if wait:
            # ROS中执行运动
            self.gripper_group.go(wait=True)

    def open_gripper(self, wait=True):
        self.move_gripper(0.04, wait=wait)
        rospy.loginfo("%s | 夹爪打开完成！" % self.arm_name)

    def close_gripper(self, wait=True):
        self.move_gripper(0.0, wait=wait)
        rospy.loginfo("%s | 夹爪闭合完成！" % self.arm_name)

    def target_distance(self, target_pose: Pose):
        x = target_pose.position.x
        y = target_pose.position.y
        return math.sqrt(x * x + y * y)

    def move_straight(self, target_pose: Pose, speed_scale=1.0, retry = False):
        if retry: rospy.loginfo("%s | 直线运动重试中..." % self.arm_name)
        waypoints = [target_pose]
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints, 0.01, avoid_collisions=True  # 路径点列表  # 步长（米）
        )
        # 执行轨迹
        if fraction < 1.0:
            rospy.loginfo(
                "%s | 规划失败，仅完成%.1f%%路径" % (self.arm_name, fraction * 100)
            )
            if retry:
                return -1
            else:
                dis = self.target_distance(target_pose)
                if dis > 0.3:
                    target_pose = self.tune_target_pose(target_pose, extend=True)
                else:
                    target_pose = self.tune_target_pose(target_pose, extend=False)
                rc = self.move_straight(target_pose, speed_scale=speed_scale, retry=True)
                return rc
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
            z_height = max(pick_point[2] - 0.08, 0.01) 
        else:
            z_height = max(pick_point[2] - 0.04, 0.01)
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
        self.move_to_start()
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
        z_height = max(pick_point[2] - 0.08, 0.01)
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
        if self.arm_name == 'panda_right':
            start_pose = position_euler_to_pose(x=0.6088363745155371
                                               ,y=0.5664727788152062
                                               ,z=0.6838833715005419
                                               ,roll=-2.9011359150910314
                                               ,pitch=-1.2380923126610726
                                               ,yaw=0.5436518860902592)
        elif self.arm_name == 'panda_left':
            start_pose = position_euler_to_pose(x=0.6088363745155371
                                               ,y=-0.5664727788152062
                                               ,z=0.6838833715005419
                                               ,roll=-2.9011359150910314
                                               ,pitch=-1.2380923126610726
                                               ,yaw=-0.5436518860902592)
        rc = self.move_straight(start_pose, 0.2)
        if rc < 0:
            return -1
        if sync:
            barrier.wait()
        rospy.loginfo("%s | 运动到铺平中间位置" % self.arm_name)
        if self.arm_name == 'panda_right':
            end_pose = position_euler_to_pose(x=0.5908389366233705
                                            ,y=-0.
                                            ,z=0.08329330047535127
                                            ,roll=-3.130197047279865
                                            ,pitch=-0.203768386711177
                                            ,yaw=-0.5516549050579803)
        elif self.arm_name == 'panda_left':
            end_pose = position_euler_to_pose(x=0.5908389366233705
                                            ,y=0.
                                            ,z=0.08329330047535127
                                            ,roll=-3.130197047279865
                                            ,pitch=-0.203768386711177
                                            ,yaw=0.5516549050579803)
        rc = self.move_straight(end_pose, 0.2)
        if rc < 0:
            return -1
        if sync:
            barrier.wait()
        rospy.loginfo("%s | 运动到铺平结束位置" % self.arm_name)
        if self.arm_name == 'panda_right':
            end_pose = position_euler_to_pose(x=0.5908389366233705
                                            ,y=-0.20047243605215895
                                            ,z=0.08329330047535127
                                            ,roll=-3.130197047279865
                                            ,pitch=-0.203768386711177
                                            ,yaw=-0.5516549050579803)
        elif self.arm_name == 'panda_left':
            end_pose = position_euler_to_pose(x=0.5908389366233705
                                            ,y=0.20047243605215895
                                            ,z=0.08329330047535127
                                            ,roll=-3.130197047279865
                                            ,pitch=-0.203768386711177
                                            ,yaw=0.5516549050579803)
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

    def pick_and_drop(self, pick_point):
        picking_height = 0.3
        rospy.loginfo(f"{self.arm_name} | 捡起点：{pick_point}")
        rospy.loginfo("%s | 运动到起始位置" % self.arm_name)
        self.move_to_start()
        self.open_gripper()
        rospy.loginfo("%s | 运动到抓取位置" % self.arm_name)
        current_joints = copy.copy(self.START_POSITION)
        current_joints[0] += math.atan2(pick_point[1], pick_point[0])
        self.move_to_joints(current_joints) 
        current_pose = self.current_pose
        pick_pose = Pose()
        pick_pose.position.x = pick_point[0]
        pick_pose.position.y = pick_point[1]
        z_height = max(pick_point[2] - 0.08, 0.01) 
        rospy.loginfo("%s | 运动到抓取高度 %s" % (self.arm_name, z_height))
        pick_pose.position.z = z_height
        pick_pose.orientation = current_pose.orientation
        rc = self.move_straight(pick_pose)
        if rc < 0:
            return -1
        rospy.loginfo("%s | 关闭夹爪" % self.arm_name)
        self.close_gripper()
        rospy.loginfo("%s | 运动到提起位置" % self.arm_name)
        pick_pose.position.z = picking_height
        rc = self.move_straight(pick_pose)
        if rc < 0:
            return -1
        rospy.loginfo("%s | 运动到丢下位置" % self.arm_name)
        drop_pose = Pose()
        drop_pose.position.x = 0.75
        drop_pose.position.y = 0.0
        drop_pose.position.z = 0.75
        drop_pose.orientation.x = -0.8870233918125237
        drop_pose.orientation.y = -1.3366887322203058e-05
        drop_pose.orientation.z = -0.461724486616894
        drop_pose.orientation.w = 2.5633932447909884e-05
        rc = self.move_straight(drop_pose, 0.2)
        if rc < 0:
            return -1
        rospy.sleep(2)
        rospy.loginfo("%s | 打开夹爪" % self.arm_name)
        self.open_gripper()
        rospy.sleep(2)
        rospy.loginfo("%s | 收回机械臂" % self.arm_name)
        self.move_to_home()
    
    def reset(self):
        self.open_gripper()
        self.move_to_home()
        rospy.loginfo("%s | 复位完成！" % self.arm_name)

    def to_vr(self):
        self.open_gripper()
        self.move_to_vr()
        rospy.loginfo("%s | move to vr完成！" % self.arm_name)

    def follow(self, target_pose:Pose, wait=False):
        if wait:
            self.move_straight(target_pose, retry=True)
        else:
            waypoints = [target_pose]
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints, 0.01, avoid_collisions=True  # 路径点列表  # 步长（米）
            )
            if fraction == 1.0:
                rospy.loginfo("%s | follow！" % self.arm_name)
                joint_trajectory = plan.joint_trajectory
                joint_state = JointState()
                joint_state.header.stamp = rospy.Time.now()
                joint_state.name = joint_trajectory.joint_names
                joint_state.position = joint_trajectory.points[-1].positions
                joint_state.velocity = []
                joint_state.effort = []
                self.joint_states_pub.publish(joint_state)
            else:
                rospy.loginfo("%s | 轨迹计算失败" % self.arm_name)