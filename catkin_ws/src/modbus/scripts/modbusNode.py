#!/usr/bin/env python3

import socket
import sre_compile

class ModbusNode():
    def __init__(self, serverIP: str, portNum: int = 50002) -> None:
        self.serverIP = serverIP
        self.port = portNum
        #with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            #s.connect((self.serverIP, self.port))
        msg: str = f"0x{0x03}" + str(0xff)+ str(0x00)+ str(0x00)+ str(0x05)
            #s.send(msg)
            #data = s.recv(1024)
        print (msg)
        #print(f"Received {data!r}")
