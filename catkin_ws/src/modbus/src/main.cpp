#include <modbusTalker.hpp>
#include <iostream>

int main(int argc, char **argv)
{
	try
	{
		modbus::ModbusTalker mbus("10.42.0.2", 50009);
		std::cout << mbus.sendMsg("0x030xff0x000x000x05") << std::endl;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	return 0;
}
