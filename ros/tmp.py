from franka_pickup import Manipulator
from geometry_msgs.msg import Pose
import rospy
from transforms3d.quaternions import mat2quat
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


rospy.init_node("tmp")
panda_right = Manipulator("panda_right")
# panda_right.open_gripper()
# panda_right.move_to_start()


current_pose = panda_right.current_pose
current_pos = current_pose.position
print(f"Current position: {current_pos.x}, {current_pos.y}, {current_pos.z}")
current_quat = current_pose.orientation
roll, pitch, yaw = euler_from_quaternion([current_quat.x, current_quat.y, current_quat.z, current_quat.w])
print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

# roll = math.pi
# pitch = -math.pi / 4
# yaw = math.pi / 4

# my_pose = Pose()
# my_pose.position.x = 0.4
# my_pose.position.y = 0.2
# my_pose.position.z = 0.5
# target_quat = quaternion_from_euler(roll, pitch, yaw)
# my_pose.orientation.x = target_quat[0]
# my_pose.orientation.y = target_quat[1]
# my_pose.orientation.z = target_quat[2]
# my_pose.orientation.w = target_quat[3]
# rc = panda_right.move_straight(my_pose, 0.2)
# print(rc)