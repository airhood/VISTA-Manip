import struct
from dataclasses import dataclass

@dataclass
class SystemStatus:
    rpi_uart: int  # 0 = disconnected, 1 = connected

    @staticmethod
    def from_bytes(data: bytes) -> 'SystemStatus':
        rpi_uart, = struct.unpack('<H', data[:2])
        return SystemStatus(rpi_uart=rpi_uart)

system_status = SystemStatus(rpi_uart=0)