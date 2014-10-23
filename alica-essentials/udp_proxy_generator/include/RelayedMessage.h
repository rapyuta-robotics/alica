#ifndef RELAYEDMESSAGE_H_
#define RELAYEDMESSAGE_H_

#include <iostream>

#include <boost/regex.hpp>

using namespace std;

class RelayedMessage
{
public:
	RelayedMessage(string topic, string message, string options);
	~RelayedMessage();

	string getRosMessageHandler();

	string Topic;
	string FullName;
	string BaseName;
	string NameSpace;
	string OptionsString;

	int Ros2UdpQueueLength;
	int Udp2RosQueueLength;

	unsigned int Id;

	string getRosCallBackName;
	string getRosClassName;

	bool UseRosTcp;
};

#endif
