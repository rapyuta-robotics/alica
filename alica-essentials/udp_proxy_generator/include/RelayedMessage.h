#pragma once 

#include <functional>
#include <iostream>

#include <boost/regex.hpp>
#include <cstdint>

class RelayedMessage
{
public:
    RelayedMessage(std::string topic, std::string message, std::string options, std::string sendReceive);
    ~RelayedMessage();

    std::string getRosMessageHandler();
    std::string getRosJavaMessageHandler();

    std::string Topic;
    std::string FullName;
    std::string BaseName;
    std::string NameSpace;
    std::string OptionsString;
    std::string SendReceiveString;

    std::string FullNameJava;

    int Ros2UdpQueueLength;
    int Udp2RosQueueLength;

    std::hash<std::string> hash;
    uint32_t Id;

    std::string getRosCallBackName();
    std::string getRosJavaCallBackName();
    std::string getRosClassName();
    std::string getPublisherName();

    bool UseRosTcp;
};

