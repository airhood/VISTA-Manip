from raspberry_utils.uart import UART
import struct
from typing import Sequence

uart = UART('/dev/ttyACM0', 115200)

COMMAND_SET_SERVO_POS = 0x01
COMMAND_TEST = 0xFF

uart.register_queue(COMMAND_SET_SERVO_POS)
uart.register_queue(COMMAND_TEST)

def sendPacket(command: int, data: bytes) -> None:
    """Send a packet to arduino consisting of a command and data. Non-blocking."""
    uart.send_packet(command, data)

def send_servo_positions(positions: Sequence[int]) -> None:
    """Send servo positions to arduino. Non-blocking."""
    data = b''.join(struct.pack('<H', p) for p in positions)
    uart.send_packet(0x01, data)

def test() -> bool:
    """Send health check to arduino and wait for response. Blocking."""
    return uart.health_check(timeout=1.0)

def waitForInit() -> bool:
    for _ in range(5):
        if test():
            return True
    return False

waitForInit()