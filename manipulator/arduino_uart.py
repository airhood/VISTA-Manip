import serial
import struct

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

COMMAND_SET_SERVO_POS = 0x01;

def sendServoPositions(servo_positions):
    data = b''.join(struct.pack('<H', p) for p in servo_positions)
    sendSerialCommand(COMMAND_SET_SERVO_POS, data)

def sendSerialCommand(command, data):
    length = len(data) + 1
    crc = command
    for b in data:
        crc ^= b
    
    packet = bytes([0xFF, length, command]) + bytes([command]) + data + bytes([crc])
    ser.write(packet)