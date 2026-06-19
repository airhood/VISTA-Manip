from . import arduino_uart
from typing import Sequence

servo_positions = [2048, 2048, 2048, 2048, 2048, 0]

def set_servo(servo_idx: int, position: int):
    servo_positions[servo_idx] = position
    arduino_uart.send_servo_positions(servo_positions)

def set_servos(positions: Sequence[int]):
    if len(positions) != 6:
        raise ValueError(f'Invalid servo positions. Position sequence length mismatch: {len(positions)}')
    servo_positions = positions
    arduino_uart.send_servo_positions(servo_positions)