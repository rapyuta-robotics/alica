#include "RelayedMessage.h"


//Alternative hasing function
//http://stackoverflow.com/questions/98153/whats-the-best-hashing-algorithm-to-use-on-a-stl-string-when-using-hash-map
//These guys says it works well... colpa sua
/*unsigned int
hash(
    const char* s,
    unsigned int seed = 0)
{
    unsigned int hash = seed;
    while (*s)
    {
        hash = hash * 101  +  *s++;
    }
    return hash;
}*/

RelayedMessage::RelayedMessage(string topic, string message, string options)
{
	this->Ros2UdpQueueLength = 5;
	this->Udp2RosQueueLength = 5;
	this->Topic = topic;
	this->OptionsString = options;

	this->Id = hash(topic);

	int lastSlash = message.find_last_of('/');
	if (lastSlash == string::npos)
	{
		//throw new Exception("unqualified Message Name: "+qualifiedName);
		cout << "No Namespace given, assuming std_msgs" << endl;
		this->BaseName = message;
		this->NameSpace = "std_msgs/";
		this->FullName = "std_msgs/" + message;
	}
	else
	{
		this->BaseName = message.substr(lastSlash + 1);
		this->NameSpace = message.substr(0, lastSlash + 1);
		this->FullName = message;
	}

	this->UseRosTcp = (options.find("tcpros") != string::npos);

	boost::regex Ros2UdpQueueRE("Ros2UdpQueueLength\\s*=\\s*([0-9]+)");
	boost::regex Udp2RosQueueRE("Ros2UdpQueueLength\\s*=\\s*([0-9]+)");

	if (boost::regex_match(options, Ros2UdpQueueRE))
	{
		boost::smatch m;
		boost::regex_search(options, m, Ros2UdpQueueRE);
		this->Ros2UdpQueueLength = std::stoi(m[1]);
	}
	if (boost::regex_match(options, Udp2RosQueueRE))
	{
		boost::smatch m;
		boost::regex_search(options, m, Udp2RosQueueRE);
		this->Udp2RosQueueLength = std::stoi(m[1]);
	}
}

RelayedMessage::~RelayedMessage()
{
}

string RelayedMessage::getRosCallBackName()
{
	return string("onRos") + BaseName + to_string(Id);
}
string RelayedMessage::getRosClassName()
{
	string ret = FullName;
	while(ret.find("/") != string::npos) {
		ret.replace(ret.find("/"), 1, "::");
	}

	return ret;
}

string RelayedMessage::getPublisherName()
{
	return string("pub")+to_string(Id);
}

string RelayedMessage::getRosMessageHandler()
{
	string ret = string("void ") + getRosCallBackName() + "(const ros::MessageEvent<" + getRosClassName() + ">& event) {\n";
	ret += "\tif(0 == event.getPublisherName().compare(ownRosName)) return;\n";
	ret += "uint8_t* buffer = NULL;\n";
	ret += "\tconst " + getRosClassName() + "::ConstPtr& message = event.getMessage();\n";
	ret += "\ttry{\n";
	ret += "\t\tuint32_t serial_size = ros::serialization::serializationLength(*message);\n";

	ret += "\t\tbuffer = new uint8_t[serial_size+sizeof(size_t)];\n";

	ret += "\t\tros::serialization::OStream stream(buffer+sizeof(size_t), serial_size);\n";

	ret += "\t\t*((size_t*)buffer) = " + to_string(Id) + "ul;\n";

	ret += "\t\tros::serialization::serialize(stream, *message);\n";

	ret += "\t\t// write message to UDP\n";
	ret += "\t\tinsocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(size_t)),destEndPoint);\n";
	ret += "\t} catch(std::exception& e) {\n";
	ret +=
			"\t\tROS_ERROR_STREAM_THROTTLE(2,\"Exception while sending UDP message:\"<<e.what()<< \" Discarding message!\");\n";

	ret += "\t}\n";
	ret += "\tif(buffer!=NULL) delete[] buffer;\n";
	ret += "}\n";
	return ret;
}

