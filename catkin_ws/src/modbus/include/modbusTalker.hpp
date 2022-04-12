#pragma once

/**
 * @author Thomas Kikkert
 *
 * @brief class form communicating with the modbus
 **/

#include <string>
#include <stdexcept>
#include <arpa/inet.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

namespace modbus
{
    class ModbusTalker
    {
        public:
	/**
	 * @param serverIP the ip of the server example "127.0.0.1"
	 * @param port the port where the data wil be send and recieved form 
	 **/
        ModbusTalker(std::string serverIP, int16_t port = 50009);

	/**
	 * @param hexMsg is the hex encode msg
	 *
	 * @brief
	 * the msg will be send to the given server:port.
	 *
	 * might give runtime_error if it can't connecto to the server 
	 **/
        std::string sendMsg(std::string hexMsg);

        protected:
	std::string ip = "";
        int port = 0;
    };
};
