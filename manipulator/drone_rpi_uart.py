from raspberry_utils.uart import UART
import serial

uart = UART('/dev/ttyACM0', 115200)

COMMAND_TEST = 0xFF

def sendPacket(command: int, data: bytes) -> None:
    """Send a packet to drone rpi consisting of a command and data. Non-blocking."""
    uart.send_packet(command, data)

def test() -> bool:
    """Send health check to arduino and wait for response. Blocking."""
    return uart.health_check(timeout=1.0)

def waitForInit() -> bool:
    for _ in range(5):
        if test():
            return True
    return False

waitForInit()