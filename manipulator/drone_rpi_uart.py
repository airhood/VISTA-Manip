from raspberry_utils.uart import UART
import serial
import struct

uart = UART(serial.Serial('/dev/ttyACM1', 115200, timeout=1))

COMMAND_TEST = 0xFF

async def sendCommand(command: int, data: bytes):
    uart.sendSerialPacket(command, data)

async def test():
    await uart.sendSerialPacket(COMMAND_TEST, b'test')

    command, data = await uart.receiveResponse(
        tinmeout=1.0,
        mode="reliable"
    )

    if command == COMMAND_TEST:
        return True
    
    return False
