import numpy as np

try:
    import roboticstoolbox as rtb
    from roboticstoolbox import ET
    from spatialmath import SE3
except ImportError:
    raise ImportError("pip install roboticstoolbox-python spatialmath-python")


# ─────────────────────────────────────────────
# 링크 길이 (단위: meter) - Unity 시뮬레이터 기준
#
# 구조:
#   (0) Link(0.036)
#   (1) Joint[0]
#   (2) Link(0.041)
#   (3) Joint[1]
#   (4) Link(0.128)
#   (5) FixedAngleJoint(Z, 90°)  ← ㄱ자 꺾임
#   (6) Link(0.024)
#   (7) Joint[2]
#   (8) Link(0.124)
#   (9) Joint[3]
#  (10) Link(0.066)
#  (11) Joint[4]
#  (12) Link(0.000)
#  (13) FixedAngleJoint(X, 0°)
# ─────────────────────────────────────────────
DEFAULT_LINK_LENGTHS = {
    "l0":  0.036,    # base ~ Joint[0]
    "l1":  0.0405,   # Joint[0] ~ Joint[1]
    "l2a": 0.128,    # Joint[1] ~ FixedAngleJoint (꺾이기 전)
    "l2b": 0.024,    # FixedAngleJoint ~ Joint[2] (꺾인 후)
    "l3":  0.124,    # Joint[2] ~ Joint[3]
    "l4":  0.06404,  # Joint[3] ~ Joint[4]
    "l5":  0.13906,  # Joint[4] ~ EndEffector
}

# joint limit (단위: radian)
DEFAULT_JOINT_LIMITS = [
    (-np.pi/2, np.pi/2),   # Joint[0]
    (-np.pi/2, np.pi/2),   # Joint[1]
    (-np.pi/2, np.pi/2),   # Joint[2]
    (-np.pi/2, np.pi/2),   # Joint[3]
    (-np.pi/2, np.pi/2),   # Joint[4]
]


class IKSolver:
    def __init__(self, link_lengths: dict = None, joint_limits: list = None):
        _ll = link_lengths if link_lengths is not None else DEFAULT_LINK_LENGTHS
        _jl = joint_limits if joint_limits is not None else DEFAULT_JOINT_LIMITS
        self.joint_limits = _jl
        self.robot = self._build_robot(_ll, _jl)

    def _build_robot(self, ll: dict, jl: list) -> rtb.ERobot:
        """
        Unity 왼손 좌표계 기준, 각 joint local frame에서:
          링크 방향: +Y, +Y, +Y, FixedJoint(Z,90°), -X, -X, -X, -X
          ET 순서: 이동(link) → 회전(joint)

        Unity q=0 end-effector 참조값: (-0.351, 0.205, 0)
        """
        e = (
            ET.ty(ll["l0"])                   # Link(0.036) +Y  base → Joint[0]
          * ET.Ry(qlim=jl[0])                 # Joint[0] Y축 회전
          * ET.ty(ll["l1"])                    # Link(0.0405) +Y  Joint[0] → Joint[1]
          * ET.Rz(qlim=jl[1])                 # Joint[1] Z축 회전
          * ET.ty(ll["l2a"])                   # Link(0.128) +Y  Joint[1] → FixedJoint
          * ET.Rz(-np.pi/2)                   # FixedAngleJoint(Z, 90°)
          * ET.ty(-ll["l2b"])                  # Link(0.024) → world -X
          * ET.Rz(qlim=jl[2])                 # Joint[2] Z축 회전
          * ET.ty(-ll["l3"])                   # Link(0.124) → world -X
          * ET.Rz(qlim=jl[3])                 # Joint[3] Z축 회전
          * ET.ty(-ll["l4"])                   # Link(0.06404) → world -X
          * ET.Rx(qlim=jl[4])                 # Joint[4] X축 회전
          * ET.ty(-ll["l5"])                   # Link(0.13906) → world -X
        )

        return rtb.ERobot(e, name="5dof_arm")

    def solve(self, x: float, y: float, z: float) -> list[float] | None:
        """
        target position (x, y, z) → joint angles (degree)
        실패 시 None 반환
        """
        T_target = SE3(x, y, z)
        target = np.array([x, y, z])

        best_q = None
        best_error = float('inf')

        for _ in range(30):
            sol = self.robot.ik_LM(
                T_target,
                joint_limits=True,
                mask=[1, 1, 1, 0, 0, 0],
                q0=np.random.uniform(
                    [b[0] for b in self.joint_limits],
                    [b[1] for b in self.joint_limits]
                )
            )
            q_rad = sol[0]
            fk_pos = self.robot.fkine(q_rad).t
            error = np.linalg.norm(target - fk_pos)
            if error < best_error:
                best_error = error
                best_q = q_rad

        if best_error < 2e-3:
            return np.degrees(best_q).tolist()
        return None