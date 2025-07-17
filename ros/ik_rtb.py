import roboticstoolbox as rtb
from typing import List
from spatialmath import SE3

class RtbKinematics:
    def __init__(self):
        # Make a Panda robot
        self.robot = rtb.models.Panda()
        # Get the ETS of the Panda
        self.ets = self.robot.ets()
        # Make an IK solver
        # https://petercorke.github.io/robotics-toolbox-python/IK/ik.html
        # self.solver = rtb.IK_QP()
        return

    def compute_ee_pose(self, qpos: List) -> SE3:
        Tep = self.robot.fkine(qpos)
        return Tep

    def compute_ik(self, ee_pose: SE3, init_qpos: List) -> List:
        # sol = self.solver.solve(self.ets, ee_pose, init_qpos)
        # sol_q = sol.q
        # sol_q = self.ets.ik_LM(ee_pose, init_qpos)[0]
        sol_q = self.ets.ik_NR(ee_pose, init_qpos)[0]
        return sol_q

if __name__ == "__main__":
    kin = RtbKinematics()
    # Make a goal pose
    init_q = [-0.00018403243177298324, -0.7853874869460222, -0.0001951762413934528, -2.3560972799573334, 0.00038120054520713475, 1.5705772726795735, 0.7858247790430386]
    ee_pose = kin.compute_ee_pose(init_q)
    print(ee_pose)
    # Solve the IK problem
    init_q[0] = -1
    test_cnt = 100
    import time
    tic = time.time()
    for _ in range(test_cnt):
        sol_q = kin.compute_ik(ee_pose, init_q)
    print("Each ik costs: ", (time.time() - tic) / test_cnt, " seconds")
    print(sol_q)
