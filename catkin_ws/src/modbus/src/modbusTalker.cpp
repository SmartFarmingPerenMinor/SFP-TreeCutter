#include <modbusTalker.hpp>
#include <iostream>
#include <cerrno>
#include <cstring>

using namespace modbus;

ModbusTalker::ModbusTalker(std::string serverIP, int16_t port)
{
    this->ip = serverIP;
    this->port = port;
};

std::string ModbusTalker::sendMsg(std::string hexMsg)
{
    std::string result = "";
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    char buffer[1024] = {0};
    std::string errorMsg = "";

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
	errorMsg = "can't create a Socket:";
	errorMsg.append(std::strerror(errno));
        throw std::runtime_error(errorMsg);
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(this->port);

    if (inet_pton(AF_INET, this->ip.c_str(), &serv_addr.sin_addr)<= 0)
    {
	errorMsg = "Invalid address/ Address not supported: ";
	errorMsg.append(std::strerror(errno));
        throw std::runtime_error(errorMsg);
    }

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr))< 0) {
	errorMsg = "Connection Failed: ";
	errorMsg.append(std::strerror(errno));
        throw std::runtime_error(errorMsg);
    }
    send(sock, hexMsg.c_str() + '\n', hexMsg.size() + 2, 0);
    read(sock, buffer, 1024);
    close(sock);

    result.append(buffer);
    return result;
};
