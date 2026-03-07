import numpy as np
from utils.IK_solver import IKSolver


def unity_to_rh(x: float, y: float, z: float) -> tuple[float, float, float]:
    """
    Unity 왼손 좌표계 → 오른손 좌표계 변환
    Unity: (x, y, z) → RH: (x, y, -z)
    """
    return x, y, -z


ik = IKSolver(
    link_lengths={
        "l0":  0.036,
        "l1":  0.0405,
        "l2a": 0.128,
        "l2b": 0.024,
        "l3":  0.124,
        "l4":  0.06404,
        "l5":  0.13906,
    },
    joint_limits=[(-np.pi, np.pi)] * 5,
)

# 테스트할 목표 좌표 (단위: meter)
test_targets = [
    (-0.10, 0.35, 0.00),
    (-0.15, 0.30, 0.05),
    ( 0.00, 0.40, 0.00),
    (-0.20, 0.25, 0.00),
    (-0.05, 0.38, 0.03),
    (0.07430609, 0.1232, -0.02988586),
]

print(f"{'Target (x, y, z)':<28} {'Joint Angles (deg)':<50} {'Status'}")
print("-" * 90)

for ux, uy, uz in test_targets:
    x, y, z = unity_to_rh(ux, uy, uz)
    angles = ik.solve(x, y, z)
    if angles is None:
        print(f"({ux:.3f}, {uy:.3f}, {uz:.3f}){'':10} IK_FAIL")
        continue

    # FK로 검증
    q_rad = np.radians(angles)
    fk_pos = ik.robot.fkine(q_rad).t
    error = np.linalg.norm(np.array([x, y, z]) - fk_pos)
    status = "OK" if error < 2e-3 else f"WARN ({error*1000:.1f}mm)"

    angles_str = ", ".join(f"{a:7.2f}°" for a in angles)
    print(f"({ux:.3f}, {uy:.3f}, {uz:.3f})  [{angles_str}]  {status}")