//
// Created by marci on 17.04.16.
//

#ifndef SUPPLEMENTARY_WRAPPEDMESSAGE_H
#define SUPPLEMENTARY_WRAPPEDMESSAGE_H

#include <iostream>
#include <functional>

#include <boost/regex.hpp>
#include <cstdint>

using namespace std;

class WrappedMessage {
public:
    WrappedMessage(const string& topic, const string& wrappedMessage, const string& message, const string& options,
            const string& sendReceive);
    ~WrappedMessage();

    string topic, message, wrappedMessage, options, sendReceive;

    int Ros2UdpQueueLength;
    int Udp2RosQueueLength;

    std::hash<std::string> hash;
    uint32_t Id;

    string getRosCallBackName();
    string getWrappedRosClassName();
    string getPublisherName();
    string getRosMessageHandler();
    string getRosWrappedMessageHandler();

    bool UseRosTcp;

    string getRosClassName();

    string getRosWrappedCallBackName();

    string getWrappedPublisherName();
};

#endif  // SUPPLEMENTARY_WRAPPEDMESSAGE_H
