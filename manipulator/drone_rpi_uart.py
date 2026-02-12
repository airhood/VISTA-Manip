from raspberry_utils.uart import UART
import serial
import struct

uart = UART(serial.Serial('/dev/ttyACM1', 115200, timeout=1))

COMMAND_TEST = 0xFF

def sendCommand(command: int, data: bytes):
    uart.sendSerialCommand(command, data)
