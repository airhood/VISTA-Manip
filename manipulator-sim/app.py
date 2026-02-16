from .manip_sim_server import ManipSimServer

server = ManipSimServer(host="127.0.0.1", port=5005)
server.startServer()

server.sendServoPositions([10, 20, 30, 40, 50, 60])

import time
time.sleep(60)

server.close()