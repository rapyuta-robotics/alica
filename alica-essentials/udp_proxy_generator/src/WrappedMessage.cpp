#include "WrappedMessage.h"

// Alternative hashing function
// http://stackoverflow.com/questions/98153/whats-the-best-hashing-algorithm-to-use-on-a-stl-string-when-using-hash-map
// These guys says it works well... colpa sua
uint32_t hash32(const char* s, unsigned int seed = 0)
{
    unsigned int hash = seed;
    while (*s) {
        hash = hash * 101 + *s++;
    }
    return hash;
}

std::string get(std::string arg)
{
    int lastSlash = arg.find_last_of(':');
    return arg.substr(lastSlash + 1);
}

WrappedMessage::WrappedMessage(const std::string& topic, const std::string& wrappedMessage, const std::string& message, const std::string& options, const std::string& sendReceive)
{
    this->Ros2UdpQueueLength = 5;
    this->Udp2RosQueueLength = 5;
    this->topic = topic;
    this->options = options;
    this->sendReceive = sendReceive;

    this->Id = hash32(topic.c_str());

    int lastSlash = message.find_last_of('/');
    if (lastSlash == std::string::npos) {
        // throw new Exception("unqualified Message Name: "+qualifiedName);
        std::cout << "No Namespace given, assuming std_msgs" << std::endl;
        this->message = message;
        this->wrappedMessage = "ttb_msgs/";
    } else {
        this->message = message;
        this->wrappedMessage = wrappedMessage;
    }

    this->UseRosTcp = (options.find("tcpros") != std::string::npos);

    boost::regex Ros2UdpQueueRE("Ros2UdpQueueLength\\s*=\\s*([0-9]+)");
    boost::regex Udp2RosQueueRE("Ros2UdpQueueLength\\s*=\\s*([0-9]+)");

    if (boost::regex_match(options, Ros2UdpQueueRE)) {
        boost::smatch m;
        boost::regex_search(options, m, Ros2UdpQueueRE);
        this->Ros2UdpQueueLength = std::stoi(m[1]);
    }
    if (boost::regex_match(options, Udp2RosQueueRE)) {
        boost::smatch m;
        boost::regex_search(options, m, Udp2RosQueueRE);
        this->Udp2RosQueueLength = std::stoi(m[1]);
    }
}

WrappedMessage::~WrappedMessage() {}

std::string WrappedMessage::getRosCallBackName()
{
    return std::string("onRos") + get(getRosClassName()) + std::to_string(Id);
}

std::string WrappedMessage::getRosWrappedCallBackName()
{
    return std::string("onRos") + get(getWrappedRosClassName()) + std::to_string(Id);
}

std::string WrappedMessage::getWrappedRosClassName()
{
    std::string ret = this->wrappedMessage;
    while (ret.find("/") != std::string::npos) {
        ret.replace(ret.find("/"), 1, "::");
    }

    return ret;
}

std::string WrappedMessage::getRosClassName()
{
    std::string ret = this->message;
    while (ret.find("/") != std::string::npos) {
        ret.replace(ret.find("/"), 1, "::");
    }

    return ret;
}

std::string WrappedMessage::getPublisherName()
{
    std::string topicChanged = this->topic;
    replace(topicChanged.begin(), topicChanged.end(), '/', '_');
    return std::string("pub") + get(getRosClassName()) + topicChanged;
}

std::string WrappedMessage::getWrappedPublisherName()
{
    std::string topicChanged = this->topic;
    replace(topicChanged.begin(), topicChanged.end(), '/', '_');
    return std::string("pub") + get(getWrappedRosClassName()) + topicChanged;
}

std::string WrappedMessage::getRosMessageHandler()
{
    std::string ret = std::string("void ") + getRosCallBackName() + "(const ros::MessageEvent<" + getRosClassName() + ">& event) {\n";
    ret += "\t" + getWrappedRosClassName() + " message;\n";
    ret += "\tmessage.receiverId = robotID;\n";
    ret += "\tmessage.msg = *event.getMessage();\n";
    ret += "\t" + getWrappedPublisherName() + ".publish(message);\n";
    ret += "}\n";
    return ret;
}

std::string WrappedMessage::getRosWrappedMessageHandler()
{
    std::string ret = std::string("void ") + getRosWrappedCallBackName() + "(const ros::MessageEvent<" + getWrappedRosClassName() + ">& event) {\n";
    ret += "\tconst " + getWrappedRosClassName() + "::ConstPtr& message = event.getMessage();\n";
    ret += "\tif(message->receiverId == robotID && event.getPublisherName().compare(ros::this_node::getName()) != 0)\n";
    ret += "\t{";
    ret += "\t\t" + getPublisherName() + ".publish(message->msg);\n";
    ret += "\t}\n";
    ret += "}";
    return ret;
}
