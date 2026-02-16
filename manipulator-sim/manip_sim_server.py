import socket
import threading

class ManipSimServer:
    def __init__(self, host="127.0.0.1", port=5005, enable_log=True):
        self.host = host
        self.port = port
        self.enable_log = enable_log
        self.server = None
        self.conn = None
        self.running = False
        self.accept_thread = None

    def log(self, *args):
        if self.enable_log:
            print("[ManipSimServer]", *args)

    def startServer(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        
        self.running = True
        self.accept_thread = threading.Thread(target=self._acceptLoop, daemon=True)
        self.accept_thread.start()
        
        self.log(f"Server started on {self.host}:{self.port}")

    def _acceptLoop(self):
        connection_count = 0
        
        while self.running:
            try:
                if connection_count == 0:
                    self.log("Waiting for Unity connection...")
                else:
                    self.log("Waiting for next connection...")
                
                conn, addr = self.server.accept()
                connection_count += 1
                
                # 기존 연결 정리
                if self.conn:
                    try:
                        self.conn.close()
                        self.log("Previous connection closed")
                    except:
                        pass
                
                self.conn = conn
                self.log(f"Connected: {addr} (connection #{connection_count})")
                
            except Exception as e:
                if self.running:
                    self.log(f"Accept error: {e}")
                break

    def sendServoPositions(self, angles):
        if self.conn is None:
            self.log("No Unity connection.")
            return False

        try:
            message = ",".join(map(str, angles)) + "\n"
            self.conn.sendall(message.encode())
            return True
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            self.log(f"Send failed (connection lost): {e}")
            self.conn = None
            return False

    def close(self):
        self.running = False
        
        if self.conn:
            try:
                self.conn.close()
            except:
                pass
            self.conn = None
            
        if self.server:
            try:
                self.server.close()
            except:
                pass
            self.server = None
            
        self.log("Server closed")