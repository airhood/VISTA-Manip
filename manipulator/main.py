import RPi.GPIO as GPIO
import atexit
import threading
from . import arduino_uart
from . import drone_rpi_uart
from . import status_led

def initSystem():
    GPIO.setmode(GPIO.BOARD)

    atexit.register(GPIO.cleanup)

    arduino_uart.init()
    drone_rpi_uart.init()

def main():
    initSystem()

    threading.Event().wait()

if __name__ == "__main__":
    main()