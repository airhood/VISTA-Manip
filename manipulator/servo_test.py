import threading
from . import arduino_uart
from .main import initSystem

NUM_SERVOS = 6

def home_servos_zero() -> None:
    arduino_uart.send_servo_positions([1024] * NUM_SERVOS)

def main():
    initSystem()
    home_servos_zero()
    threading.Event().wait()

if __name__ == "__main__":
    main()
