#!/usr/bin/env python3

import socket

class ModbusNode():
    def __init__(self, serverIP: str, portNum: int = 50002) -> None:
        self.serverIP = serverIP
        self.port = portNum
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.serverIP, self.port))
            s.send((0x03<< 8*4) + int( "FF000005",16))
            data = s.recv(1024)

        print(f"Received {data!r}")
