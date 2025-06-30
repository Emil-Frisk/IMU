import socket
from time import sleep
import threading
import logging
import sys
from imu import ISM330DLC


class TCPSocketServer():
    def __init__(self, port=7001, host="0.0.0.0", logger=None):
        self.host = host
        self.port = port
        self.is_running = False
        self.clients = {}
        self.server_socket = None
        self.logger= self._setup_logger(logger)

    def _start_thread(self, f, *args):
        thread = threading.Thread(
                        args=args,
                        target=f
                    )
        thread.daemon = True
        thread.start()

    def _setup_logger(self, logger):
        """Set up logger with sensible defaults"""
        if logger:
            return logger
        
        # Create default logger
        logger_name = f"TCPServer-{self.host}:{self.port}"
        default_logger = logging.getLogger(logger_name)
        
        # Only configure if not already configured
        if not default_logger.handlers:
            default_logger.setLevel(logging.INFO)
            
            # Console handler
            console_handler = logging.StreamHandler(sys.stdout)
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                datefmt='%H:%M:%S'
            )
            console_handler.setFormatter(formatter)
            default_logger.addHandler(console_handler)
        
        return default_logger
    
    def start(self):
        self.server_socket = self._create_server()
        self.sensor = ISM330DLC(address=0x6A)
        if self.server_socket:
            self._start_thread(self._listen_for_connections)

    def _create_server(self):
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.bind((self.host,self.port))
            server_socket.listen(5)
            self.logger.info(f"Host listening on {self.host}:{self.port}")
            self.is_running = True
            return server_socket
        except Exception as e:
            self.logger.error(f"Failed to create server: {e}") 
            return None

    def _listen_for_connections(self):
        try:
            self.server_socket.settimeout(1.0)
            while self.is_running:
                try:
                    client_socket, client_addr = self.server_socket.accept() 
                    self._start_thread(self._handle_client_connection, client_socket, client_addr)
                    self.logger.info("Started thread to handle client connection!")
                    self.clients[client_socket] = {"address": client_addr }
                except socket.timeout:
                    continue

            self.logger.info("Server is exiting listening for connections")
        except:
            self.logger.error(f"Something went wrong while listening for connections. Context: {client_socket, client_addr}")
    
    def _handle_client_connection(self, client_socket, client_address):
        try:
            client_socket.settimeout(1.0)
            while self.is_running:
                try:
                    data = client_socket.recv(1024)
                    if data:
                        data = data.decode("utf-8")
                        action = self.extract_part("action=", data)
                        if not action:
                            raise Exception("No action given.")
                        if action == "r_xl":
                            pitch, roll = self.sensor.read_accelerometer()
                            message = f"message={pitch},{roll}|".encode("utf-8")
                            client_socket.sendall(message)
                    else:
                        self.logger.info(f"Client: {client_address} has closed their connection")
                except socket.timeout:
                    continue  
            self.logger.info("Client is exiting listening for connections")
        except Exception as e:
            self.logger.error(f"Something went wrong while handling client connection. {e}")

    def _close_client_sockets(self):
        try:
            for socket, client_info in self.clients.items():
                socket.close()
                self.logger.info(f"Close client connection: {client_info['address']}")
                self.clients[socket] = None
        except Exception as e:
            self.logger.error(f"Something went wrong while closing client connection. {e}")

    def close(self):
        try:
            self.is_running = False
            self._close_client_sockets()
            self.server_socket.close()
        except Exception as e:
            self.logger.error(f"Something went wrong while closing the socket server. {e}")
            
    def extract_part(self,part, message):
        start_idx = message.find(part)
        if start_idx == -1:
            return False

        start_idx += len(part)
        pipe_idx = message.find("|", start_idx)
        if pipe_idx == -1:
            return False

        return  message[start_idx:pipe_idx]

if __name__ == "__main__":
    so_tcp_server = TCPSocketServer()
    so_tcp_server.start()
    while True:
        sleep(3600)
