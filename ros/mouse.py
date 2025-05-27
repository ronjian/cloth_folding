import pyautogui
import rospy
from manipulator import Manipulator
from geometry_msgs.msg import Pose
import time

# 获取当前鼠标位置
init_x, init_y = pyautogui.position()
rospy.loginfo(f"当前鼠标位置: X={init_x}, Y={init_y}")

def main():
    panda_right = Manipulator("panda_right")
    panda_right.move_to_start()
    # init_pose = panda_right.current_pose
    init_pose = Pose()
    init_pose.position.x = 0.3068996400631961
    init_pose.position.y = 1.5825692895146504e-05
    init_pose.position.z = 0.4868641264189588
    init_pose.orientation.x = -0.9999999975158107
    init_pose.orientation.y = -2.211139319355152e-05
    init_pose.orientation.z = 4.807790928831443e-05
    init_pose.orientation.w = -4.6561563415960625e-05
    while True:
        current_x, current_y = pyautogui.position()
        X = current_x - init_x
        Y = current_y - init_y
        rospy.loginfo(f"鼠标当前数值: X={X}, Y={Y}")
        target_pose = Pose()
        target_pose.position.x = init_pose.position.x - Y * 0.001
        target_pose.position.y = init_pose.position.y - X * 0.001
        target_pose.position.z = init_pose.position.z
        rospy.loginfo(f"当前位姿: X={init_pose.position.x}, Y={init_pose.position.y}, Z={init_pose.position.z}")
        rospy.loginfo(f"目标位置: X={target_pose.position.x}, Y={target_pose.position.y}, Z={target_pose.position.z}")
        target_pose.orientation = init_pose.orientation
        tic = time.time() 
        panda_right.follow(target_pose)
        rospy.loginfo(f"移动时间: {time.time() - tic:.2f}秒")

if __name__ == "__main__":
    try:
        rospy.init_node('mouse_control_node', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass