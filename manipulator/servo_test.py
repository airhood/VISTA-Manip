import threading
from . import arduino_uart
from .main import initSystem

NUM_SERVOS = 6

def set_all_servos_zero() -> None:
    arduino_uart.send_servo_positions([0] * NUM_SERVOS)

def main():
    initSystem()
    set_all_servos_zero()
    threading.Event().wait()

if __name__ == "__main__":
    main()
