import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import ET
from utils.IK_solver import IKSolver
from utils.coordinate_transform import unity_to_rh

LINK_LENGTHS = {
    "l0":  0.036,
    "l1":  0.0405,
    "l2a": 0.128,
    "l2b": 0.024,
    "l3":  0.124,
    "l4":  0.06404,
    "l5":  0.13906,
}

def build_unity_fk(ll):
    """solver와 동일한 구조 — Unity FK 검증용"""
    jl = [(-np.pi, np.pi)] * 5
    e = (
        ET.ty(ll["l0"])   * ET.Ry(qlim=jl[0])
      * ET.ty(ll["l1"])   * ET.Rz(qlim=jl[1])
      * ET.ty(ll["l2a"])  * ET.Rz(-np.pi/2)
      * ET.ty(-ll["l2b"]) * ET.Rz(qlim=jl[2])
      * ET.ty(-ll["l3"])  * ET.Rz(qlim=jl[3])
      * ET.ty(-ll["l4"])  * ET.Rx(qlim=jl[4])
      * ET.ty(-ll["l5"])
    )
    return rtb.ERobot(e, name="unity_fk")

ik = IKSolver(link_lengths=LINK_LENGTHS, joint_limits=[(-np.pi, np.pi)] * 5)
unity_robot = build_unity_fk(LINK_LENGTHS)

test_targets = [
    (-0.10, 0.35,  0.00),
    (-0.15, 0.30,  0.05),
    ( 0.00, 0.40,  0.00),
    (-0.20, 0.25,  0.00),
    (-0.05, 0.38,  0.03),
    ( 0.07430609, 0.07348678, -0.02988586),
]

print(f"{'Unity Target':<28} {'Unity FK Result':<28} {'Error (m)':<12} {'Status'}")
print("-" * 85)

for ux, uy, uz in test_targets:
    x, y, z = unity_to_rh(ux, uy, uz)
    angles = ik.solve(x, y, z)

    if angles is None:
        print(f"({ux:.4f}, {uy:.4f}, {uz:.4f}){'':8} IK_FAIL")
        continue

    # 부호 반전 없이 그대로
    q = np.radians(angles)
    fk = unity_robot.fkine(q).t

    error = np.linalg.norm(np.array([ux, uy, uz]) - fk)
    status = "OK" if error < 2e-3 else f"WARN ({error*1000:.1f}mm)"

    print(f"({ux:.4f}, {uy:.4f}, {uz:.4f})  ({fk[0]:.4f}, {fk[1]:.4f}, {fk[2]:.4f})  {error:.6f}  {status}")
    print(f"  joints: [{', '.join(f'{a:6.1f}°' for a in angles)}]")