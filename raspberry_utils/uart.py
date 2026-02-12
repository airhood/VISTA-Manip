import asyncio
import serial
from typing import Tuple, Optional

class UART:
    MIN_PACKET_SIZE = 4

    def __init__(self, serial: serial.Serial) -> None:
        self.ser = serial

    # data must be in binary format
    async def sendSerialCommand(self, command: int, data: bytes) -> None:
        length = len(data) + 1
        crc = command
        for b in data:
            crc ^= b
        
        packet = bytes([0xFF, length, command]) + data + bytes([crc])
        
        await asyncio.get_event_loop().run_in_executor(
            None, self.ser.write, packet
        )
        await asyncio.get_event_loop().run_in_executor(
            None, self.ser.flush
        )

    async def receiveResponse(self, timeout: float = 1.0) -> Tuple[Optional[int], Optional[bytes]]:
        start_time = asyncio.get_event_loop().time()

        self.ser.timeout = timeout
        
        while asyncio.get_event_loop().time() - start_time < timeout:
            if self.ser.in_waiting < 4: continue

            header = await asyncio.get_event_loop().run_in_executor(
                None, self.ser.read, 1
            )
            
            if header == b'\xFF':
                length_byte = await asyncio.get_event_loop().run_in_executor(
                    None, self.ser.read, 1
                )
                length = ord(length_byte)
                
                command_byte = await asyncio.get_event_loop().run_in_executor(
                    None, self.ser.read, 1
                )
                command = ord(command_byte)
                
                data = await asyncio.get_event_loop().run_in_executor(
                    None, self.ser.read, length - 1
                )
                
                crc_byte = await asyncio.get_event_loop().run_in_executor(
                    None, self.ser.read, 1
                )
                crc = ord(crc_byte)
                
                calculated_crc = command
                for b in data:
                    calculated_crc ^= b
                
                if crc == calculated_crc:
                    return command, data
            
            await asyncio.sleep(0.001)

        return None, None
    
    async def receiveResponse(self,
                              timeout: float = 1.0,
                              mode: str = "reliable" # "reliable" or "realtime"
                              ) -> Tuple[Optional[int], Optional[bytes]]:
        loop = asyncio.get_running_loop()
        start_time = loop.time()

        self.ser.timeout = timeout

        if mode == "reliable":
            buffer = bytearray()

            while loop.time() - start_time < timeout:
                chunk = await loop.run_in_executor(None, self.ser.read, self.ser.in_waiting or 1)

                buffer.extend(chunk)

                while True:
                    if len(buffer) < self.MIN_PACKET_SIZE:
                        break

                    if buffer[0] != 0xFF:
                        buffer.pop(0)
                        continue

                    length = buffer[1]
                    total_size = 1 + 1 + length + 1

                    if len(buffer) < total_size:
                        break

                    packet = buffer[:total_size]

                    command = packet[2]
                    data = packet[3:-1]
                    crc = packet[-1]

                    calculated_crc = command
                    for b in data:
                        calculated_crc ^= b
                    
                    if crc == calculated_crc:
                        del buffer[:total_size] # remove parsed command buffer
                        return command, data
                    else:
                        buffer.pop(0) # resync with offset 1
                        continue

                return None, None
        elif mode == "realtime":
            while loop.time() - start_time < timeout:
                if self.ser.in_waiting < self.MIN_PACKET_SIZE:
                    await asyncio.sleep(0.001) # 1ms
                    continue

                header = await loop.run_in_executor(None, self.ser.read, 1)
                if header != b'\xFF':
                    continue

                length_byte = await loop.run_in_executor(None, self.ser.read, 1)
                if not length_byte:
                    continue
                length = length_byte[0]

                command_byte = await loop.run_in_executor(None, self.ser.read, 1)
                if not command_byte:
                    continue
                command = command_byte[0]

                data = await loop.run_in_executor(None, self.ser.read, length - 1)
                if len(data) != length - 1:
                    continue

                crc_byte = await loop.run_in_executor(None, self.ser.read, 1)
                if not crc_byte:
                    continue
                crc = crc_byte[0]

                calculated_crc = command
                for b in data:
                    calculated_crc ^= b

                if crc == calculated_crc:
                    return command, data
                
                await asyncio.sleep(0.001)

            return None, None
        
        else:
            raise ValueError("receiveResponse mode must be 'reliable' or 'realtime'")

    
    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
