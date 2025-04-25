from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import omni
from isaacsim.core.utils.extensions import enable_extension
from omni.isaac.dynamic_control import _dynamic_control

# enable ROS bridge extension
enable_extension("isaacsim.ros1.bridge")

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

import math, os

# Note that this is not the system level rospy, but one compiled for omniverse
import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import GripperCommandActionGoal


class Arm:
    def __init__(self, arm_name):
        self.joint_states_sub = rospy.Subscriber(
            "/{}/joint_states".format(arm_name),
            JointState,
            self.joint_states_callback,
            queue_size=10,
        )
        self.gripper_goal_sub = rospy.Subscriber(
            "/{}/franka_gripper/gripper_action/goal".format(arm_name),
            GripperCommandActionGoal,
            self.gripper_goal_callback,
            queue_size=10,
        )
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.joints_pos = [p * math.pi / 180.0 for p in [0, -45, 0, -135, 0, 90, 45]]
        self.joints_name = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]
        self.gripper_pos = 0.04
        self.arm_name = arm_name

    def joint_states_callback(self, msg: JointState):
        self.joints_pos = msg.position

    def gripper_goal_callback(self, msg: GripperCommandActionGoal):
        self.gripper_pos = msg.goal.command.position

    def close(self):
        self.joint_states_sub.unregister()
        self.gripper_goal_sub.unregister()

    def update(self):
        articulation = self.dc.get_articulation("/World/{}".format(self.arm_name))
        # update joints
        for name, pos in zip(self.joints_name, self.joints_pos):
            dof = self.dc.find_articulation_dof(articulation, name)
            self.dc.set_dof_position_target(dof, pos)
        # update gripper
        self.dc.set_dof_position_target(
            self.dc.find_articulation_dof(articulation, "panda_finger_joint1"),
            self.gripper_pos,
        )
        self.dc.set_dof_position_target(
            self.dc.find_articulation_dof(articulation, "panda_finger_joint2"),
            self.gripper_pos,
        )


class RosSubscriber:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        simulation_app.update()
        current_dir = os.path.dirname(os.path.abspath(__file__))
        stage_path = os.path.join(current_dir, "franka_scene/franka_scene_1.usd")
        omni.usd.get_context().open_stage(stage_path)
        simulation_app.update()
        print("Loading stage...")
        from isaacsim.core.utils.stage import is_stage_loading

        while is_stage_loading():
            simulation_app.update()
        print("Loading Complete")
        simulation_app.update()
        self.panda_left = Arm("panda_left")
        self.panda_right = Arm("panda_right")

    def run_simulation(self):
        self.timeline.play()
        while simulation_app.is_running():
            # the actual setting the franka is done here
            simulation_app.update()
            self.panda_left.update()
            self.panda_right.update()

        # Cleanup
        rospy.signal_shutdown("franka subscriber complete")
        self.panda_left.close()
        self.panda_right.close()
        self.timeline.stop()
        simulation_app.close()


if __name__ == "__main__":
    rospy.init_node(
        "ros_subscriber", anonymous=True, disable_signals=True, log_level=rospy.ERROR
    )
    ros_subscriber = RosSubscriber()
    ros_subscriber.run_simulation()
