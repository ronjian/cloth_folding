import rospy
from unity_robotics_demo_msgs.msg import XrOrigin
from manipulator import Manipulator
from threading import Thread
from geometry_msgs.msg import Pose
import tf.transformations as tf
import time
import math


class DeltaData:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ready = False
        self.euler = None
    
    def __str__(self):
        return f"Delta Position: ({self.x}, {self.y}, {self.z}), Ready: {self.ready}"

class XROriginData:
    def __init__(self):
        self.left_controller = ControllerData()
        self.right_controller = ControllerData()
        self.main_camera = ControllerData()
        self.keys = KeysData()

    def __str__(self):
        return (
            "=== Left Controller ===\n"
            f"{self.left_controller}\n\n"
            "=== Right Controller ===\n"
            f"{self.right_controller}\n\n"
            "=== Main Camera ===\n"
            f"{self.main_camera}\n\n"
            "=== Keys ===\n"
            f"{self.keys}"
        )

class ControllerData:
    def __init__(self):
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.rot_x = 0.0
        self.rot_y = 0.0
        self.rot_z = 0.0
        self.rot_w = 0.0
        self.euler_x = 0.0
        self.euler_y = 0.0
        self.euler_z = 0.0

    def __str__(self):
        return (
            f"Position: ({self.pos_x}, {self.pos_y}, {self.pos_z})\n"
            f"Rotation: ({self.rot_x}, {self.rot_y}, {self.rot_z}, {self.rot_w})"
        )

class KeysData:
    def __init__(self):
        self.left_trigger = 0.0
        self.right_trigger = 0.0
        self.left_x = False
        self.left_y = False
        self.right_a = False
        self.right_b = False
        self.left_menu = False
        self.right_meta = False
        self.left_gripper = False
        self.right_gripper = False
        self.left_thumbstick_x = 0.0
        self.left_thumbstick_y = 0.0
        self.right_thumbstick_x = 0.0
        self.right_thumbstick_y = 0.0

    def __str__(self):
        return (
            f"Triggers: L={self.left_trigger}, R={self.right_trigger}\n"
            f"Buttons: X={self.left_x}, Y={self.left_y}, A={self.right_a}, B={self.right_b}\n"
            f"Menu: L_Menu={self.left_menu}, R_Meta={self.right_meta}\n"
            f"Grippers: L={self.left_gripper}, R={self.right_gripper}\n"
            f"Thumbsticks: L=({self.left_thumbstick_x}, {self.left_thumbstick_y}), "
            f"R=({self.right_thumbstick_x}, {self.right_thumbstick_y})"
        )

def unity_to_ros_position(unity_pos):
    """
    将Unity位置坐标转换为ROS坐标
    参数:
        unity_pos: Unity中的位置 (x, y, z)
    返回:
        ROS中的位置 (x, y, z)
    """
    ros_x = unity_pos[2]  # Unity Z -> ROS X
    ros_y = -unity_pos[0]  # Unity X -> ROS Y (取反)
    ros_z = unity_pos[1]  # Unity Y -> ROS Z
    return (ros_x, ros_y, ros_z)

class ROSSubscriber:
    def __init__(self):
        self.vr_data = XROriginData()
        self.vr_subscriber = rospy.Subscriber('/xr_origin', XrOrigin, self.vr_callback)

        self.left_delta = DeltaData()
        self.right_delta = DeltaData()

        self.panda_left = Manipulator("panda_left")
        self.panda_right = Manipulator("panda_right")
        self.move_arm_to_vr_position()

        Thread(target=self.control, daemon=True).start()

        Thread(target=self.follow_vr_gripper, daemon=True).start()

        Thread(target=self.follow_left, daemon=True).start()
        Thread(target=self.follow_right, daemon=True).start()

        while not rospy.is_shutdown():
            time.sleep(0.1)
        
    def vr_callback(self, msg: XrOrigin):
        self._update_controller_data(self.vr_data.left_controller, msg.left_controller)
        self._update_controller_data(self.vr_data.right_controller, msg.right_controller)
        self._update_controller_data(self.vr_data.main_camera, msg.main_camera)
        
        keys = msg.keys
        self.vr_data.keys.left_trigger = keys.left_trigger
        self.vr_data.keys.right_trigger = keys.right_trigger
        self.vr_data.keys.left_x = keys.left_x
        self.vr_data.keys.left_y = keys.left_y
        self.vr_data.keys.right_a = keys.right_a
        self.vr_data.keys.right_b = keys.right_b
        self.vr_data.keys.left_menu = keys.left_menu
        self.vr_data.keys.right_meta = keys.right_meta
        self.vr_data.keys.left_gripper = keys.left_gripper
        self.vr_data.keys.right_gripper = keys.right_gripper
        self.vr_data.keys.left_thumbstick_x = keys.left_thumbstick_x
        self.vr_data.keys.left_thumbstick_y = keys.left_thumbstick_y
        self.vr_data.keys.right_thumbstick_x = keys.right_thumbstick_x
        self.vr_data.keys.right_thumbstick_y = keys.right_thumbstick_y

    def _update_controller_data(self, target: ControllerData, source):
        target.pos_x = source.pos_x
        target.pos_y = source.pos_y
        target.pos_z = source.pos_z
        target.rot_x = source.rot_x
        target.rot_y = source.rot_y
        target.rot_z = source.rot_z
        target.rot_w = source.rot_w
        target.euler_x = source.euler_x / 180.0 * math.pi
        target.euler_y = source.euler_y / 180.0 * math.pi
        target.euler_z = source.euler_z / 180.0 * math.pi

    def set_delta(self, delta: DeltaData, arm: Manipulator, vr_controller: ControllerData):
        arm.to_vr()
        ros_xyz = unity_to_ros_position(
            (vr_controller.pos_x,
            vr_controller.pos_y,
            vr_controller.pos_z)
        )
        current_pose = arm.current_pose
        delta.x = current_pose.position.x - ros_xyz[0]
        delta.y = current_pose.position.y - ros_xyz[1]
        delta.z = current_pose.position.z - ros_xyz[2]

        delta.euler = tf.euler_from_quaternion([current_pose.orientation.x, 
                                                current_pose.orientation.y, 
                                                current_pose.orientation.z, 
                                                current_pose.orientation.w], 'szxy')
        delta.ready = True

    def control(self):
        while True:
            if self.vr_data.keys.left_x:
                self.set_delta(self.left_delta, self.panda_left, self.vr_data.left_controller)
            
            if self.vr_data.keys.right_a:
                self.set_delta(self.right_delta, self.panda_right, self.vr_data.right_controller)
            
            if self.vr_data.keys.left_y:
                self.left_delta = DeltaData()

            if self.vr_data.keys.right_b:
                self.right_delta = DeltaData()

    def move_arm_to_vr_position(self):
        t_left = Thread(target=self.panda_left.to_vr, args=())
        t_left.start()
        t_right = Thread(target=self.panda_right.to_vr, args=())
        t_right.start()
        t_left.join()
        t_right.join()
        return

    def cal_target_pose(self, delta: DeltaData, vr_controller: ControllerData):
        ros_xyz = unity_to_ros_position(
            (vr_controller.pos_x,
            vr_controller.pos_y,
            vr_controller.pos_z)
        )
        target_pose = Pose()
        target_pose.position.x = ros_xyz[0] + delta.x
        target_pose.position.y = ros_xyz[1] + delta.y
        target_pose.position.z = ros_xyz[2] + delta.z

        euler = [vr_controller.euler_x, vr_controller.euler_y, vr_controller.euler_z]
        quat = tf.quaternion_from_euler(-euler[2] + delta.euler[0], euler[1] + delta.euler[1], euler[0] + delta.euler[2], 'szxy')
        target_pose.orientation.x = quat[0]
        target_pose.orientation.y = quat[1]
        target_pose.orientation.z = quat[2]
        target_pose.orientation.w = quat[3]
        return target_pose

    def follow_left(self):
        while True:
            if self.left_delta.ready:
                left_target_pose = self.cal_target_pose(self.left_delta, self.vr_data.left_controller)
                self.panda_left.follow(left_target_pose, False)
    
    def follow_right(self):
        while True:
            if self.right_delta.ready:
                right_target_pose = self.cal_target_pose(self.right_delta, self.vr_data.right_controller)
                self.panda_right.follow(right_target_pose, False)
    
    def follow_vr_gripper(self):
        left_close = False
        right_close = False
        while True:
            if self.vr_data.keys.left_trigger > 0.5 and not left_close:
                self.panda_left.close_gripper(False)
                left_close = True
            if self.vr_data.keys.right_trigger > 0.5 and not right_close:
                self.panda_right.close_gripper(False)
                right_close = True
            if self.vr_data.keys.left_trigger <= 0.5 and left_close:
                self.panda_left.open_gripper(False)
                left_close = False
            if self.vr_data.keys.right_trigger <= 0.5 and right_close:
                self.panda_right.open_gripper(False)
                right_close = False

if __name__ == '__main__':
    rospy.init_node('vr', anonymous=True)
    ros_subscriber = ROSSubscriber()
