from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import omni
from isaacsim.core.utils.extensions import enable_extension
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Gf, UsdGeom
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.prims import XFormPrim

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

class Camera:
    def __init__(self):
        CAMERA_STAGE_PATH = "/Camera"
        # Creating a Camera prim
        camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(CAMERA_STAGE_PATH, "Camera"))
        self.xform_api = UsdGeom.XformCommonAPI(camera_prim)
        self.xform_api.SetTranslate(Gf.Vec3d(0.8, 0, 1.8))
        self.xform_api.SetRotate((0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        camera_prim.GetHorizontalApertureAttr().Set(21)
        camera_prim.GetVerticalApertureAttr().Set(16)
        camera_prim.GetProjectionAttr().Set("perspective")
        camera_prim.GetFocalLengthAttr().Set(8)
        camera_prim.GetFocusDistanceAttr().Set(88)

        simulation_app.update()

        # Creating an on-demand push graph with cameraHelper nodes to generate ROS image publishers
        keys = og.Controller.Keys
        (ros_camera_graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": "/ROS_Camera",
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                    ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                    ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                    ("cameraHelperRgb", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                    ("cameraHelperInfo", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                    ("cameraHelperDepth", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                    ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                    ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                    ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                    ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                    ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                    ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("createViewport.inputs:viewportId", 0),
                    ("cameraHelperRgb.inputs:frameId", "sim_camera"),
                    ("cameraHelperRgb.inputs:topicName", "rgb"),
                    ("cameraHelperRgb.inputs:type", "rgb"),
                    ("cameraHelperInfo.inputs:frameId", "sim_camera"),
                    ("cameraHelperInfo.inputs:topicName", "camera_info"),
                    ("cameraHelperInfo.inputs:type", "camera_info"),
                    ("cameraHelperDepth.inputs:frameId", "sim_camera"),
                    ("cameraHelperDepth.inputs:topicName", "depth"),
                    ("cameraHelperDepth.inputs:type", "depth"),
                    ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_STAGE_PATH)]),
                ],
            },
        )

        # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
        og.Controller.evaluate_sync(ros_camera_graph)

        simulation_app.update()
        self.frame = 0

    def rotate(self):
        # Rotate camera by 0.5 degree every frame
        self.xform_api.SetRotate((0, 0, self.frame / 4.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        self.frame = self.frame + 1

class RosSubscriber:
    def __init__(self):
        self.simulation_context = SimulationContext(stage_units_in_meters=1.0)
        simulation_app.update()
        current_dir = os.path.dirname(os.path.abspath(__file__))
        stage_path = os.path.join(current_dir, "franka_scene/franka_scene_1.usd")

        omni.usd.get_context().open_stage(stage_path)

        self.camera = Camera()

        # Need to initialize physics getting any articulation..etc
        self.simulation_context.initialize_physics()

        self.simulation_context.play()

        simulation_app.update()
        self.panda_left = Arm("panda_left")
        self.panda_right = Arm("panda_right")

    def run_simulation(self):
        while simulation_app.is_running():
            simulation_app.update()
            # Run with a fixed step size
            self.simulation_context.step(render=True)

            if self.simulation_context.is_playing():
                self.camera.rotate()
                self.panda_left.update()
                self.panda_right.update()

                xform_prim = XFormPrim("/World/panda_left")
                position = xform_prim.get_world_poses()[0]
                quat = xform_prim.get_world_poses()[1]
                print("Position (X, Y, Z):", position)
                print("Orientation (W, X, Y, Z):", quat)


        # Cleanup
        rospy.signal_shutdown("franka subscriber complete")
        self.panda_left.close()
        self.panda_right.close()
        self.simulation_context.stop()
        simulation_app.close()


if __name__ == "__main__":
    rospy.init_node(
        "ros_subscriber", anonymous=True, disable_signals=True, log_level=rospy.ERROR
    )
    ros_subscriber = RosSubscriber()
    ros_subscriber.run_simulation()
