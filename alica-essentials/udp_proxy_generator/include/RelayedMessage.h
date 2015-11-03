#ifndef RELAYEDMESSAGE_H_
#define RELAYEDMESSAGE_H_

#include <iostream>
#include <functional>

#include <boost/regex.hpp>

using namespace std;

class RelayedMessage
{
public:
	RelayedMessage(string topic, string message, string options);
	~RelayedMessage();

	string getRosMessageHandler();
	string getRosJavaMessageHandler();

	string Topic;
	string FullName;
	string BaseName;
	string NameSpace;
	string OptionsString;

	string FullNameJava;

	int Ros2UdpQueueLength;
	int Udp2RosQueueLength;

	std::hash<std::string> hash;
	size_t Id;

	string getRosCallBackName();
	string getRosJavaCallBackName();
	string getRosClassName();
	string getPublisherName();

	bool UseRosTcp;
};

#endif
