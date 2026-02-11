import serial
import struct

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

COMMAND_SET_SERVO_POS = 0x01
COMMAND_TEST = 0xFF

def sendServoPositions(servo_positions):
    data = b''.join(struct.pack('<H', p) for p in servo_positions)
    sendSerialCommand(COMMAND_SET_SERVO_POS, data)

# data must be in binary format
def sendSerialCommand(command, data):
    length = len(data) + 1
    crc = command
    for b in data:
        crc ^= b
    
    packet = bytes([0xFF, length, command]) + data + bytes([crc])
    ser.write(packet)