#include <modbusTalker.hpp>
#include <iostream>

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

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        throw std::runtime_error("can't create a Socket");
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(this->port);

    if (inet_pton(AF_INET, this->ip.c_str(), &serv_addr.sin_addr)<= 0) 
    {
        throw std::runtime_error("Invalid address/ Address not supported");
    }

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr))< 0) {
        throw std::runtime_error("Connection Failed");
    }
    send(sock, hexMsg.c_str(), hexMsg.size() + 1, 0);
    read(sock, buffer, 1024);
    close(sock);

    result.append(buffer);
    return result;
};
