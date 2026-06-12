from . import servo

def cli_control():
    while True:
        cmd = input().split()
        if len(cmd) == 6:
            servo_positions = list(map(int, cmd))
            servo.set_servos(servo_positions)
        elif len(cmd) == 2:
            servo_idx, servo_position = map(int, cmd)
            servo.set_servo(servo_idx, servo_position)
        elif len(cmd) == 1:
            if cmd[0] == 'exit':
                break
