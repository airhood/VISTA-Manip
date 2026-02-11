import arduino_uart
import time

while True:
    arduino_uart.sendSerialCommand(arduino_uart.COMMAND_TEST, b"hello world!")

    if arduino_uart.ser.in_waiting > 0:
        line = arduino_uart.ser.readline().decode(errors='ignore').strip()
        if line.startswith("[TEST]"):
            print("DEBUG:", line)

    time.sleep(0.02)