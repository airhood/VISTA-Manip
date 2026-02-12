from raspberry_utils.uart import UART
import serial
import struct
from typing import Sequence

uart = UART(serial.Serial('/dev/ttyACM0', 115200, timeout=1))

COMMAND_SET_SERVO_POS = 0x01
COMMAND_TEST = 0xFF

def sendCommand(command: int, data: bytes) -> None:
    uart.sendSerialCommand(command, data)

def sendServoPositions(servo_positions: Sequence[int]) -> None:
    data = b''.join(struct.pack('<H', p) for p in servo_positions)
    uart.sendSerialCommand(COMMAND_SET_SERVO_POS, data)

async def test() -> bool:
    await uart.sendSerialCommand(COMMAND_TEST, b'')

    command, data = await uart.receiveResponse(
        timeout=1.0,
        mode="reliable"
    )

    if command == COMMAND_TEST:
        return True
    
    return False
