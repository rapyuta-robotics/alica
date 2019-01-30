#pragma once 

#include <functional>
#include <iostream>

#include <boost/regex.hpp>
#include <cstdint>

class WrappedMessage
{
public:
    WrappedMessage(const std::string& topic, const std::string& wrappedMessage, const std::string& message, const std::string& options, const std::string& sendReceive);
    ~WrappedMessage();

    std::string topic, message, wrappedMessage, options, sendReceive;

    int Ros2UdpQueueLength;
    int Udp2RosQueueLength;

    std::hash<std::string> hash;
    uint32_t Id;

    std::string getRosCallBackName();
    std::string getWrappedRosClassName();
    std::string getPublisherName();
    std::string getRosMessageHandler();
    std::string getRosWrappedMessageHandler();

    bool UseRosTcp;

    std::string getRosClassName();

    std::string getRosWrappedCallBackName();

    std::string getWrappedPublisherName();
};

