import numpy as np
from spatialmath import SE3
from utils.IK_solver import IKSolver

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
    joint_limits=[(-np.pi/2, np.pi/2)] * 5,
)

# FK로 도달 가능한 위치를 직접 샘플링해서 테스트
test_configs = [
    [0.0,    0.3,   -0.3,   0.3,  0.0],
    [0.5,   -0.3,    0.3,  -0.3,  0.0],
    [0.0,    0.5,   -0.5,   0.5,  0.0],
    [-0.5,   0.3,   -0.3,   0.3,  0.0],
    [0.3,    0.0,    0.0,   0.0,  0.5],
]

print(f"{'Target (from FK)':<30} {'IK→FK Result':<30} {'Error (m)':<12} {'Status'}")
print("-" * 90)

for cfg in test_configs:
    q_ref = np.array(cfg)

    # 이 joint 설정으로 FK → target position 생성
    T_ref = ik.robot.fkine(q_ref)
    target = T_ref.t

    # IK로 역산
    T_target = SE3(target[0], target[1], target[2])
    sol = ik.robot.ik_LM(T_target, joint_limits=True, mask=[1, 1, 1, 0, 0, 0])
    q_rad, residual = sol[0], sol[4]

    if residual >= 2e-3:
        print(f"({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}){'':10} IK_FAIL")
        continue

    T_fk = ik.robot.fkine(q_rad)
    fk_pos = T_fk.t
    error = np.linalg.norm(target - fk_pos)
    status = "OK" if error < 2e-3 else "WARN"

    target_str = f"({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})"
    fk_str     = f"({fk_pos[0]:.3f}, {fk_pos[1]:.3f}, {fk_pos[2]:.3f})"
    print(f"{target_str:<30} {fk_str:<30} {error:<12.6f} {status}")