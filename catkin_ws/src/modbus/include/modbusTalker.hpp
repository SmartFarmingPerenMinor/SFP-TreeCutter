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
#include <thread>
#include <mutex>
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace modbus
{
    typedef enum{
        readCoils = 1,
        readDiscreteInput = 2,
        readHoldingRegisters = 3,
        readInputRegister = 4,
        writeSingleCoil = 5,
        writeSingleRegister = 6,
        writeMulitepleCoils = 15,
        writeMulitepleRegister = 16,
        readFileRecord = 20,
        writeFileRecord = 21,
        r_wMultipleRegister = 23,
        readFIFOQueue = 24,
        readDeviceIdentification = 43
    } modMsgList_t;

    class ModbusTalker
    {
        public:
	    /**
	    * @param serverIP the ip of the server example "127.0.0.1"
	    * @param port the port where the data wil be send and recieved form 
	    **/
        ModbusTalker(std::string serverIP, int16_t port = 50009);
        
        /**
        * @brief
        * Destructor for killing the threads
        **/
        ~ModbusTalker();

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
        static void readCoils(bool &isRunning);
        static void readDiscreteInput(bool &isRunning);
        static void readHoldingRegisters(bool &isRunning);
        static void readInputRegister(bool &isRunning); 
        static void writeSingleCoil(bool &isRunning);
        static void writeSingleRegister(bool &isRunning);
        static void writeMulitepleCoils(bool &isRunning);
        static void writeMulitepleRegister(bool &isRunning);
        static void readFileRecord(bool &isRunning);
        static void writeFileRecord(bool &isRunning);
        static void r_wMultipleRegister(bool &isRunning);
        static void readFIFOQueue(bool &isRunning);
        static void readDeviceIdentification(bool &isRunning);
                
        static void callback(const std_msgs::String::ConstPtr& msg, const modMsgList_t &msgType, std::mutex &publishMutex, ros::Publisher &publisher);

        protected:
        bool enableThreads=0;
	    std::string ip = "";
        int port = 0;
    };
};
