import threading
import queue
import serial
import struct
import time
from typing import Tuple, Optional, Sequence


class UART:
    MIN_PACKET_SIZE = 4

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.ser = serial.Serial(port, baudrate)
        self.send_queue = queue.Queue()

        self._recv_lock = threading.Lock()
        self._pending_response: Optional[Tuple[int, bytes]] = None
        self._response_event = threading.Event()

        self._send_worker = threading.Thread(target=self._send_loop, daemon=True)
        self._recv_worker = threading.Thread(target=self._recv_loop, daemon=True)
        self._send_worker.start()
        self._recv_worker.start()

    def _send_loop(self) -> None:
        while True:
            packet = self.send_queue.get()
            try:
                self.ser.write(packet)
                self.ser.flush()
            except serial.SerialException:
                pass

    def _recv_loop(self) -> None:
        buffer = bytearray()
        while True:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException:
                time.sleep(0.001)
                continue

            buffer.extend(chunk)

            while True:
                if len(buffer) < self.MIN_PACKET_SIZE:
                    break

                if buffer[0] != 0xFF:
                    buffer.pop(0)
                    continue

                length = buffer[1]
                total_size = 1 + 1 + length + 1  # 0xFF + length + command+data + crc

                if len(buffer) < total_size:
                    break

                packet = buffer[:total_size]
                command = packet[2]
                data = bytes(packet[3:-1])
                crc = packet[-1]

                calculated_crc = command
                for b in data:
                    calculated_crc ^= b

                if crc == calculated_crc:
                    del buffer[:total_size]
                    with self._recv_lock:
                        self._pending_response = (command, data)
                        self._response_event.set()
                else:
                    buffer.pop(0)  # resync
                    continue

    def _make_packet(self, command: int, data: bytes) -> bytes:
        length = len(data) + 1
        crc = command
        for b in data:
            crc ^= b
        return bytes([0xFF, length, command]) + data + bytes([crc])

    def send_packet(self, command: int, data: bytes) -> None:
        """Send a packet consisting of a command and data. Non-blocking."""
        self.send_queue.put(self._make_packet(command, data))

    def receive_response(self, timeout: float = 1.0) -> Tuple[Optional[int], Optional[bytes]]:
        """
        Wait for a response packet from Arduino.
        Blocking — call from a separate thread or infrequently.
        Returns (command, data) or (None, None) on timeout.
        """
        self._response_event.clear()
        with self._recv_lock:
            self._pending_response = None

        if self._response_event.wait(timeout=timeout):
            with self._recv_lock:
                result = self._pending_response
                self._pending_response = None
            return result if result is not None else (None, None)

        return None, None

    def health_check(self, timeout: float = 1.0) -> bool:
        """Send health check and wait for response. Blocking."""
        self._response_event.clear()
        with self._recv_lock:
            self._pending_response = None

        self.send_packet(0xFF, b'\x01')

        if self._response_event.wait(timeout=timeout):
            with self._recv_lock:
                result = self._pending_response
                self._pending_response = None
            if result is not None:
                command, data = result
                return command == 0xFF and data == b'\x01'
        return False

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()