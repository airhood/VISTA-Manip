from raspberry_utils.uart import UART

uart = None

COMMAND_DEBUG_PRINT = 0x10
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

def onDebugPrint(data: bytes):
    print(f"[Arduino] {data.decode('ascii', errors='replace')}")

def init():
    global uart
    uart = UART('/dev/ttyAMA3', 115200)
    uart.register_handler(COMMAND_DEBUG_PRINT, onDebugPrint)
    uart.register_queue(COMMAND_TEST)

    return wait_for_UART_init()