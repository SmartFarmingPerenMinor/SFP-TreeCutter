#!/usr/bin/env python3

from dataclasses import dataclass

"""
basic msg format of the modbus
"""
@dataclass
class ModbusMsg():
    """
    erro code:
    0 no errors
    """
    error = 0

    def __init__(self, functionCode = 0):
        self.fcode = functionCode

    def setMsg(self, msg)-> None:
        self.msg = msg

    def getMsg(self):
        if (self.error != 0):
            return {"fcode": self.error, "msg": self.msg}
        else:
            return {"fcode": self.fcode, "msg": hex(self.msg)}

    def setErrorCode(self, number, msg)-> None:
        self.error = number
        self.msg = msg
