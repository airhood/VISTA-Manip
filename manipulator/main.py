import RPi.GPIO as GPIO
import atexit
import arduino_uart
import drone_rpi_uart
import status_led

def initSystem():
    GPIO.setmode(GPIO.BOARD)

    atexit.register(GPIO.cleanup)

    arduino_uart.init()
    drone_rpi_uart.init()

def main():
    initSystem()

if __name__ == "__main__":
    main()