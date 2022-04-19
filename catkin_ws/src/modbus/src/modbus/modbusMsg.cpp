#include "../../include/modbus/modbusMsg.hpp"

using namespace modbus;

ModbusMsg::ModbusMsg(const int8_t functionCode)
{
    this->fcode = functionCode;
}

ModbusMsg::ModbusMsg(const int8_t functionCode, const std::string hexDatastr)
{
    this->fcode = functionCode;
    this->msg = hexDatastr;
}

void ModbusMsg::setMsg(const uint16_t address, const uint16_t count)
{
	this->mbMsg.addressH = (address & 0xFF00);
	this->mbMsg.addressL = (address & 0x00FF);
	this->mbMsg.countH = (count & 0xFF00);
        this->mbMsg.countL = (count & 0x00FF);
}

void ModbusMsg::setError(const int8_t error, const std::string errorMsg)
{
    this->error=true;
    this->fcode=error;
    this->msg = errorMsg;
}

const std::string ModbusMsg::getMsg(void) const
{
    return this->msg;
}

void ModbusMsg::setHexMsg(const std::string hexMsg)
{
    this->msg = hexMsg;
}

void ModbusMsg::msgToHexStr(void)
{
    this->msg = {this->fcode};
}
