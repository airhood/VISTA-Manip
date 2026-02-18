import threading
import queue
import serial
import struct
import time
from typing import Callable, Dict, Optional, Sequence, Tuple


class UART:
    MIN_PACKET_SIZE = 4

    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.ser = serial.Serial(port, baudrate)
        self.send_queue = queue.Queue()

        self._recv_queues: Dict[int, queue.Queue] = {}
        self._handlers: Dict[int, Callable[[bytes], None]] = {}
        self._threaded_handlers: Dict[int, Callable[[bytes], None]] = {}

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
                    if command in self._handlers:
                        self._handlers[command](data)
                    elif command in self._threaded_handlers:
                        self._threaded_handlers[command][0].put(data)
                    elif command in self._recv_queues:
                        self._recv_queues[command].put(data)
                else:
                    buffer.pop(0)  # resync
                    continue

    def _make_threaded_handler_loop(self, command: int, handler: Callable[[bytes], None]):
        def loop():
            q = self._threaded_handlers[command][0]
            while True:
                data = q.get()
                handler(data)
        return loop

    def register_queue(self, command: int) -> None:
        """Register a command to be buffered. Retrieve with receive_response()."""
        self._recv_queues[command] = queue.Queue()

    def register_handler(self, command: int, handler: Callable[[bytes], None], enable_threading: bool = False) -> None:
        """
        Register a handler to be called when command is received.
        enable_threading=False: runs in recv_loop directly (fast callbacks only).
        enable_threading=True: runs in a dedicated worker thread per command.
        """
        if enable_threading:
            q = queue.Queue()
            worker = threading.Thread(target=self._make_threaded_handler_loop(command, handler), daemon=True)
            self._threaded_handlers[command] = (q, worker)
            worker.start()
        else:
            self._handlers[command] = handler

    def _make_packet(self, command: int, data: bytes) -> bytes:
        length = len(data) + 1
        crc = command
        for b in data:
            crc ^= b
        return bytes([0xFF, length, command]) + data + bytes([crc])

    def send_packet(self, command: int, data: bytes) -> None:
        """Fire-and-forget. Non-blocking."""
        self.send_queue.put(self._make_packet(command, data))

    def send_servo_positions(self, positions: Sequence[int]) -> None:
        """Send servo positions. Non-blocking."""
        data = b''.join(struct.pack('<H', p) for p in positions)
        self.send_packet(0x01, data)

    def receive_response(self, command: int, timeout: float = 1.0) -> Optional[bytes]:
        """
        Wait for a buffered response for the given command.
        Blocking. Returns data or None on timeout.
        Must have called register_queue(command) first.
        """
        q = self._recv_queues.get(command)
        if q is None:
            raise ValueError(f"Command 0x{command:02X} is not registered as a queue. Call register_queue() first.")
        try:
            return q.get(timeout=timeout)
        except queue.Empty:
            return None

    def health_check(self, timeout: float = 1.0) -> bool:
        """Send health check and wait for response. Blocking."""
        self.send_packet(0xFF, b'\x01')
        data = self.receive_response(0xFF, timeout=timeout)
        print(f"data: {data}")
        return data == b'\x01'

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()