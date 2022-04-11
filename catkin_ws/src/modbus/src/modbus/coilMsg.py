#!/usr/bin/env python3

from dataclasses import dataclass
import modbusMsg

@dataclass
class CoilsMsg():
    """
    function codes given by Universal Robots
    https://modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf
    """
    fCodeRead: hex = 0x01
    fCodeWriteS: hex = 0x05
    fCodeWriteM: hex = 0x15
    startAddress: hex = 0x0000
    lastAddress: hex = 0xFFFF
    lastCoil: hex = 2000
    minCoil: hex = 0x01

    def read(self, coilsCount: hex, address: hex) -> modbusMsg.ModbusMsg():
        return self.__genMsg(self.fCodeRead, coilsCount, address)

    def write(self, coilsCount: hex, address: hex) -> modbusMsg.ModbusMsg():
        if (coilsCount > 1):
            return self.__genMsg(self.fCodeWriteM, coilsCount, address)
        else:
            return self.__genMsg(self.fCodeWriteS, coilsCount, address)

    def __split(self, num: hex):
        return hex(num >> 8), hex(num & 0xFF)

    def __genMsg(self, code: hex, coilsCount: hex, address: hex) -> modbusMsg.ModbusMsg():
        mbus = modbusMsg.ModbusMsg(code)

        if ((coilsCount > self.lastCoil) & (coilsCount >= 1)):
            mbus.setErrorCode(-0x03, "in valid quantity")
            return mbus
        else:
            addressHigh, addressLow = self.__split(address)
            countHigh, countLow = self.__split(coilsCount)
            msg: hex = (int(addressLow, 16) << (8 * 3)) + (int(addressHigh, 16) << (8 * 2)) + (int(countHigh, 16) << 8) + int(countLow, 16)

            mbus.setMsg(msg)
            return mbus
