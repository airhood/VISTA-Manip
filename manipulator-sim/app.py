import time
import numpy as np
from utils.IK_solver import IKSolver
from .manip_sim_server import ManipSimServer

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

server = ManipSimServer(host="127.0.0.1", port=5005, ik_solver=ik)
server.startServer()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    server.close()