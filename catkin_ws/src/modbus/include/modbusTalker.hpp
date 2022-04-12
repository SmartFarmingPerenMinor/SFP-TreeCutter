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
        ModbusTalker(std::string serverIP, int16_t port = 50009);

        std::string sendMsg(std::string hexMsg);

        protected:
	std::string ip = "";
        int port = 0;
    };
};
