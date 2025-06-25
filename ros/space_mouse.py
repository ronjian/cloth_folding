# import pyautogui
import rospy
from manipulator import Manipulator
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose
import time
import pyspacemouse
import threading
import numpy as np

dx, dy, dz, d_roll, d_pitch, d_yaw = 0, 0, 0, 0, 0, 0
angle_scale = 0.24
translation_scale = 0.06
def read_spacemouse():
    assert pyspacemouse.open()
    while True:
        spacemouse_read = pyspacemouse.read()
        spacemouse_xyz_rot_np = np.array(
                    [
                        spacemouse_read.x,
                        spacemouse_read.y,
                        spacemouse_read.z,
                        spacemouse_read.roll,
                        spacemouse_read.pitch,
                        spacemouse_read.yaw,
                    ]
                )
        if np.max(np.abs(spacemouse_xyz_rot_np)) > 0.9:
            spacemouse_xyz_rot_np[np.abs(spacemouse_xyz_rot_np) < 0.6] = 0
        global dx, dy, dz, d_roll, d_pitch, d_yaw
        dx, dy, dz, d_roll, d_pitch, d_yaw = spacemouse_xyz_rot_np
        time.sleep(0.01)

def main():
    threading.Thread(target=read_spacemouse, daemon=True).start()

    arm = Manipulator("panda_left")
    arm.move_to_start()

    while True:
        current_pose = arm.current_pose
        target_pose = Pose()

        target_x = current_pose.position.x + dx * translation_scale
        target_y = current_pose.position.y + dy * translation_scale
        target_z = current_pose.position.z + dz * translation_scale
        target_pose.position.x = target_x
        target_pose.position.y = target_y
        target_pose.position.z = target_z
        # target_pose.position = current_pose.position

        current_roll, current_pitch, current_yaw = euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])
        rospy.loginfo(f"当前欧拉角: roll={current_roll:.2f}, pitch={current_pitch:.2f}, yaw={current_yaw:.2f}")
        target_roll = current_roll + d_roll * angle_scale
        target_pitch = current_pitch + d_pitch * angle_scale
        target_yaw = current_yaw + d_yaw * angle_scale
        rospy.loginfo(f"目标欧拉角: roll={target_roll:.2f}, pitch={target_pitch:.2f}, yaw={target_yaw:.2f}")
        xyzw = quaternion_from_euler(target_roll, target_pitch, target_yaw)
        target_pose.orientation.x = xyzw[0]
        target_pose.orientation.y = xyzw[1]
        target_pose.orientation.z = xyzw[2]
        target_pose.orientation.w = xyzw[3]
        # target_pose.position = current_pose.position

        tic = time.time() 
        arm.follow(target_pose, wait=False)
        rospy.loginfo(f"移动时间: {time.time() - tic:.2f}秒")

if __name__ == "__main__":
    try:
        rospy.init_node('mouse_control_node', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass