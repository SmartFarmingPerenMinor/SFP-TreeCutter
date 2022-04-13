#pragma once

/**
 * @author Thomas Kikkert
 * @brief modbus msg format
 **/

#include <string>

namespace modbus
{
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
        std::string msg = "";
        int8_t fcode = 0;
        bool error = false;
        int8_t errorCode = 0;
    };
};
