from raspberry_utils.uart import UART

uart = UART('/dev/ttyACM0', 115200)

COMMAND_TEST = 0xFF

def send_packet(command: int, data: bytes) -> None:
    """Send a packet to drone rpi consisting of a command and data. Non-blocking."""
    uart.send_packet(command, data)

def test() -> bool:
    """Send health check to arduino and wait for response. Blocking."""
    return uart.health_check(timeout=1.0)

def wait_for_UART_init() -> bool:
    for _ in range(5):
        if test():
            return True
    return False

def init():
    uart.register_queue(COMMAND_TEST)

    return wait_for_UART_init()