from raspberry_utils.uart import UART
import serial
import struct
from typing import Sequence

uart = UART(serial.Serial('/dev/ttyACM0', 115200, timeout=1))

COMMAND_SET_SERVO_POS = 0x01
COMMAND_TEST = 0xFF

async def sendPacket(command: int, data: bytes):
    await uart.sendSerialPacket(command, data)

async def sendServoPositions(servo_positions: Sequence[int]):
    data = b''.join(struct.pack('<H', p) for p in servo_positions)
    await uart.sendSerialPacket(COMMAND_SET_SERVO_POS, data)

async def test():
    await uart.sendSerialPacket(COMMAND_TEST, b'test')

    command, data = await uart.receiveResponse(
        timeout=1.0,
        mode="reliable"
    )

    if command == COMMAND_TEST:
        return True
    
    return False
