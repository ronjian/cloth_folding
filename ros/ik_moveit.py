import rospy
from manipulator import Manipulator
from geometry_msgs.msg import Pose
import numpy as np

def main():
    panda_left = Manipulator("panda_left")
    target_pose: Pose = panda_left.current_pose
    rospy.loginfo(f"当前姿态： {target_pose}")
    solved_joints = panda_left.ik(target_pose)
    rospy.loginfo(f"解算出的关节角度：{solved_joints}")
    current_joints = panda_left.current_joints
    rospy.loginfo(f"目标关节角度：{current_joints}")
    assert np.allclose(solved_joints, current_joints, rtol=0.001, atol=0.001)
    return

if __name__ == "__main__":
    try:
        rospy.init_node('ik_moveit', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass