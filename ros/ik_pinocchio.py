import numpy as np
try:
    import pinocchio as pin
except ImportError:
    print("pinocchio安装问题，可能需要设置环境变量：")
    print("export PATH=/opt/openrobots/bin:$PATH")
    print("export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH")
    print("export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH")
    print("export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH")
    print("export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH")
from typing import Dict, List
import os
from geometry_msgs.msg import Pose

def pose_msg_to_se3(pose_msg: Pose):
    """
    将 geometry_msgs.msg.Pose 转换为 pinocchio.SE3
    
    参数:
        pose_msg (geometry_msgs.msg.Pose): ROS Pose 消息
        
    返回:
        pinocchio.SE3: 对应的 SE3 变换
    """
    position = np.array([pose_msg.position.x, 
                         pose_msg.position.y, 
                         pose_msg.position.z])
    quat = pin.Quaternion(w = pose_msg.orientation.w, x = pose_msg.orientation.x, y = pose_msg.orientation.y, z = pose_msg.orientation.z)
    se3 = pin.SE3(quat, position)
    return se3

class PinKinematics:
    def __init__(self):
        # convert xacro to urdf:
        # cd /opt/ros/noetic/share/franka_description/robots/panda
        # xacro panda.urdf.xacro hand:=true gazebo:=true > /home/jiangrong/isaacsim/cloth_folding/ros/assets/franka_panda/panda.urdf
        current_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(current_dir, "assets/franka_panda/panda.urdf")
        self.pin_model: pin.Model = pin.buildModelFromUrdf(urdf_path)
        self.pin_data: pin.Data = self.pin_model.createData()
        self.frame_id_mapping: Dict[str, int] = {}
        for i, frame in enumerate(self.pin_model.frames):
            self.frame_id_mapping[frame.name] = i
        ee_frame_name = "panda_hand_tcp"
        self.ee_frame_id = self.frame_id_mapping[ee_frame_name]
    
    def compute_ee_pose(self, qpos: List) -> pin.SE3:
        qpos = np.array(qpos)
        if qpos.shape[0] == 7:
            qpos = np.concatenate([qpos, np.zeros(2)])
        pin.forwardKinematics(self.pin_model, self.pin_data, qpos)
        ee_pose: pin.SE3 = pin.updateFramePlacement(
            self.pin_model, self.pin_data, self.ee_frame_id
        )
        return ee_pose
    

    def compute_ik(self, ee_pose: pin.SE3, init_qpos: List):
        oMdes = ee_pose
        qpos = np.array(init_qpos)

        if qpos.shape[0] == 7:
            qpos = np.concatenate([qpos, np.zeros(2)])

        for k in range(200):
            pin.forwardKinematics(self.pin_model, self.pin_data, qpos)
            ee_pose = pin.updateFramePlacement(
                self.pin_model, self.pin_data, self.ee_frame_id
            )
            J = pin.computeFrameJacobian(
                self.pin_model, self.pin_data, qpos, self.ee_frame_id
            )
            iMd = ee_pose.actInv(oMdes)
            err = pin.log(iMd).vector
            # print(k, np.linalg.norm(err))
            if np.linalg.norm(err) < 1e-3:
                break

            v = J.T.dot(np.linalg.solve(J.dot(J.T) + 1e-5, err))
            qpos = pin.integrate(self.pin_model, qpos, v * 0.05)
        return list(qpos)

if __name__ == "__main__":
    kin = PinKinematics()
    init_q = [-0.00018403243177298324, -0.7853874869460222, -0.0001951762413934528, -2.3560972799573334, 0.00038120054520713475, 1.5705772726795735, 0.7858247790430386]
    ee_pose = kin.compute_ee_pose(init_q)
    print(ee_pose)
    init_q[0] = -1
    test_cnt = 100
    import time
    tic = time.time()
    for _ in range(test_cnt):
        solved_q = kin.compute_ik(ee_pose, init_q)
    print("Each ik costs: ", (time.time() - tic) / test_cnt, " seconds")
    print(solved_q)