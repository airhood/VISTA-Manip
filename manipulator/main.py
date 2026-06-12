import RPi.GPIO as GPIO
import atexit
import threading
import argparse
from . import arduino_uart
from . import drone_rpi_uart
from . import status_led
from . import cli_control

def init_system():
    GPIO.setmode(GPIO.BOARD)

    atexit.register(GPIO.cleanup)

    status_led.init_status_led()

    arduino_uart.init()
    drone_rpi_uart.init()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cli-control', action='store_true')

    args = parser.parse_args()

    init_system()

    print("system initialized")

    if args.cli_control:
        print("cli control enabled")
        cli_control.cli_control()
    else:
        threading.Event().wait()

if __name__ == "__main__":
    main()