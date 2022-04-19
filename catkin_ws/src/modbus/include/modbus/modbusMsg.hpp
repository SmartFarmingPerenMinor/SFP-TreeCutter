#pragma once

/**
 * @author Thomas Kikkert
 * @brief modbus msg format
 **/

#include <string>

namespace modbus
{
    typedef union
    {
        struct
        {
            unsigned char function_code: 8;
            unsigned char addressH: 8;
            unsigned char addressL: 8;
            unsigned char countH: 8;
            unsigned char countL: 8;
        };
        unsigned char msg[6];
    } modbusMsg_t;

    typedef union
    {
        struct
        {
            unsigned char function_code: 8;
            unsigned char addressH: 8;
            unsigned char addressL: 8;
            unsigned char countH: 8;
            unsigned char countL: 8;
            unsigned char byteCount: 8;
            unsigned char outPutsValue: 8; // Quantity of Outputs / 8, if the remainder is different of 0 ⇒ N = N+1
        };
        unsigned char msg[8];
    } modbusMsgEx_t;

    class ModbusMsg
    {
        public:
        /**
         * @brief constructor modbus msg
         * 
         * @param functionCode modbus function code
         **/
        ModbusMsg(const int8_t functionCode);

        /**
         * @brief constructor modbus msg
         * 
         * @param functionCode modbus function code
         * @param hexDatastr is modbus msg that is hex encode
         **/
        ModbusMsg(const int8_t functionCode, const std::string hexDatastr);

	/**
         * @brief setter for error
	 *
         * @param error is the error code
         * @param errorMsg is error msg
         **/
        void setError(const int8_t error, const std::string errorMsg);

	/**
	 * @param address 0x000 format 0xff00 is addr high and 0x00ff is addr low
	 * @param count 0x000 format 0xff00 is count high and 0x00ff is count low
	 **/
	void setMsg(const uint16_t address, const uint16_t count);

        /**
         * @param hexMsg modbus msg that is ready to send
         **/
        void setHexMsg(const std::string hexMsg);

        /**
         * @brief returns the generate msg
         **/
        const std::string getMsg(void) const;

        private:
        /**
         * @brief needs to be over writen
         * the default version only converts the function code
         **/
        void virtual msgToHexStr(void);

        private:
	modbusMsg_t mbMsg={};
        std::string msg = "";
        int8_t fcode = 0;
        bool error = false;
        int8_t errorCode = 0;
    };
};
