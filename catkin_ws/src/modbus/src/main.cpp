#include <modbusTalker.hpp>
#include <iostream>

int main(int argc, char **argv)
{
	try
	{
		modbus::ModbusTalker mbus("127.0.0.0");
		mbus.sendMsg("0x030xff0x000x000x05");
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	return 0;
}
