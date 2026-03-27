from raspberry_utils.uart import UART
from raspberry_utils.system_status import SystemStatus, system_status
import struct
from typing import Sequence

uart = UART('/dev/ttyACM0', 115200)

COMMAND_SET_SERVO_POS = 0x01
COMMAND_SYSTEM_STATUS_UPDATE_SYNC = 0x02
COMMAND_SYSTEM_STATUS_UPDATE_REQUEST = 0x03
COMMAND_DEBUG_PRINT = 0x10
COMMAND_TEST = 0xFF

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

def waitForUARTInit() -> bool:
    for _ in range(5):
        if test():
            return True
    return False

def onSystemStatusUpdate(data: bytes):
    global system_status
    system_status = SystemStatus.from_bytes(data)
    print(f"System Status: {system_status}")

def onDebugPrint(data: bytes):
    print(f"[Arduino] {data.decode('ascii', errors='replace')}")

def init():
    uart.register_queue(COMMAND_SET_SERVO_POS)
    uart.register_handler(COMMAND_SYSTEM_STATUS_UPDATE_SYNC, onSystemStatusUpdate)
    uart.register_handler(COMMAND_DEBUG_PRINT, onDebugPrint)
    uart.register_queue(COMMAND_TEST)

    return waitForUARTInit()