# 借鉴: https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py
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
import casadi
from pinocchio import casadi as cpin

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

        self.cmodel = cpin.Model(self.pin_model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.pin_model.nq, 1) 
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.ee_frame_id].translation - self.cTf[:3,3],
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.ee_frame_id].rotation @ self.cTf[:3,:3].T),
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.pin_model.nq)
        self.var_q_last = self.opti.parameter(self.pin_model.nq)   # for smooth
        self.param_tf = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.pin_model.lowerPositionLimit,
            self.var_q,
            self.pin_model.upperPositionLimit)
        )
        self.opti.minimize(50 * self.translational_cost + self.rotation_cost + 0.02 * self.regularization_cost + 0.1 * self.smooth_cost)

        opts = {
            'ipopt':{
                'print_level':0,
                'max_iter':100,
                'tol':1e-6,
                'acceptable_tol': 1e-3,
                'mu_strategy': 'adaptive'
            },
            'print_time':False,# print or not
            'calc_lam_p':False# https://github.com/casadi/casadi/wiki/FAQ:-Why-am-I-getting-%22NaN-detected%22in-my-optimization%3F
        }
        self.opti.solver("ipopt", opts)
        self.init_data = np.zeros(self.pin_model.nq)
    
    def compute_ee_pose(self, qpos: List) -> pin.SE3:
        qpos = np.array(qpos)
        if qpos.shape[0] == 7:
            qpos = np.concatenate([qpos, np.zeros(2)])
        pin.forwardKinematics(self.pin_model, self.pin_data, qpos)
        ee_pose: pin.SE3 = pin.updateFramePlacement(
            self.pin_model, self.pin_data, self.ee_frame_id
        )
        return ee_pose
    

    # def compute_ik(self, ee_pose: pin.SE3, init_qpos: List):
    #     self.init_data = np.array(init_qpos)
    #     self.opti.set_initial(self.var_q, self.init_data)

    #     self.opti.set_value(self.param_tf, ee_pose.homogeneous)
    #     self.opti.set_value(self.var_q_last, self.init_data) # for smooth

    #     try:
    #         _ = self.opti.solve()
    #         sol_q = self.opti.value(self.var_q)
    #     except Exception as e:
    #         sol_q = self.opti.debug.value(self.var_q)
    #     return list(sol_q)

    def compute_ik(self, ee_pose: pin.SE3, init_qpos: List):
        # 输入检查
        init_qpos = np.array(init_qpos)
        if np.any(np.isnan(init_qpos)):
            raise ValueError("Initial qpos contains NaN values")
        if np.any(np.isnan(ee_pose.homogeneous)):
            raise ValueError("Target pose contains NaN values")
        
        # 归一化旋转矩阵
        # R = ee_pose.rotation
        # U, S, Vh = np.linalg.svd(R)
        # R_normalized = U @ Vh
        # ee_pose_normalized = pin.SE3(R_normalized, ee_pose.translation)
        
        # 设置初始值和参数
        self.opti.set_initial(self.var_q, init_qpos)
        self.opti.set_value(self.param_tf, ee_pose.homogeneous)
        self.opti.set_value(self.var_q_last, init_qpos)
        
        try:
            sol = self.opti.solve()
            sol_q = self.opti.value(self.var_q)
        except Exception as e:
            print("Optimization failed:", e)
            print("Debug values:")
            print("Current q:", self.opti.debug.value(self.var_q))
            print("Translation error:", self.opti.debug.value(self.translational_cost))
            print("Rotation error:", self.opti.debug.value(self.rotation_cost))
            print("Regularization error:", self.opti.debug.value(self.regularization_cost))
            print("Smooth error:", self.opti.debug.value(self.smooth_cost))
            sol_q = self.opti.debug.value(self.var_q)  # 返回最后一次迭代结果
        
        return list(sol_q)

if __name__ == "__main__":
    kin = PinKinematics()
    init_q = [-0.00018403243177298324, -0.7853874869460222, -0.0001951762413934528, -2.3560972799573334, 0.00038120054520713475, 1.5705772726795735, 0.7858247790430386, 0.0, 0.0]
    ee_pose = kin.compute_ee_pose(init_q)
    print(ee_pose)
    init_q[0] = -1
    solved_q = kin.compute_ik(ee_pose, init_q)
    print(solved_q)