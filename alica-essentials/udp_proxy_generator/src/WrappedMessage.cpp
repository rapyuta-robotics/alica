#include "WrappedMessage.h"

// Alternative hashing function
// http://stackoverflow.com/questions/98153/whats-the-best-hashing-algorithm-to-use-on-a-stl-string-when-using-hash-map
// These guys says it works well... colpa sua
uint32_t hash32(const char* s, unsigned int seed = 0) {
    unsigned int hash = seed;
    while (*s) {
        hash = hash * 101 + *s++;
    }
    return hash;
}

string get(string arg) {
    int lastSlash = arg.find_last_of(':');
    return arg.substr(lastSlash + 1);
}

WrappedMessage::WrappedMessage(const string& topic, const string& wrappedMessage, const string& message,
        const string& options, const string& sendReceive) {
    this->Ros2UdpQueueLength = 5;
    this->Udp2RosQueueLength = 5;
    this->topic = topic;
    this->options = options;
    this->sendReceive = sendReceive;

    this->Id = hash32(topic.c_str());

    int lastSlash = message.find_last_of('/');
    if (lastSlash == string::npos) {
        // throw new Exception("unqualified Message Name: "+qualifiedName);
        cout << "No Namespace given, assuming std_msgs" << endl;
        this->message = message;
        this->wrappedMessage = "ttb_msgs/";
    } else {
        this->message = message;
        this->wrappedMessage = wrappedMessage;
    }

    this->UseRosTcp = (options.find("tcpros") != string::npos);

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

string WrappedMessage::getRosCallBackName() {
    return string("onRos") + get(getRosClassName()) + to_string(Id);
}

string WrappedMessage::getRosWrappedCallBackName() {
    return string("onRos") + get(getWrappedRosClassName()) + to_string(Id);
}

string WrappedMessage::getWrappedRosClassName() {
    string ret = this->wrappedMessage;
    while (ret.find("/") != string::npos) {
        ret.replace(ret.find("/"), 1, "::");
    }

    return ret;
}

string WrappedMessage::getRosClassName() {
    string ret = this->message;
    while (ret.find("/") != string::npos) {
        ret.replace(ret.find("/"), 1, "::");
    }

    return ret;
}

string WrappedMessage::getPublisherName() {
    string topicChanged = this->topic;
    replace(topicChanged.begin(), topicChanged.end(), '/', '_');
    return string("pub") + get(getRosClassName()) + topicChanged;
}

string WrappedMessage::getWrappedPublisherName() {
    string topicChanged = this->topic;
    replace(topicChanged.begin(), topicChanged.end(), '/', '_');
    return string("pub") + get(getWrappedRosClassName()) + topicChanged;
}

string WrappedMessage::getRosMessageHandler() {
    string ret =
            string("void ") + getRosCallBackName() + "(const ros::MessageEvent<" + getRosClassName() + ">& event) {\n";
    ret += "\t" + getWrappedRosClassName() + " message;\n";
    ret += "\tmessage.receiverId = robotID;\n";
    ret += "\tmessage.msg = *event.getMessage();\n";
    ret += "\t" + getWrappedPublisherName() + ".publish(message);\n";
    ret += "}\n";
    return ret;
}

string WrappedMessage::getRosWrappedMessageHandler() {
    string ret = string("void ") + getRosWrappedCallBackName() + "(const ros::MessageEvent<" +
                 getWrappedRosClassName() + ">& event) {\n";
    ret += "\tconst " + getWrappedRosClassName() + "::ConstPtr& message = event.getMessage();\n";
    ret += "\tif(message->receiverId == robotID && event.getPublisherName().compare(ros::this_node::getName()) != 0)\n";
    ret += "\t{";
    ret += "\t\t" + getPublisherName() + ".publish(message->msg);\n";
    ret += "\t}\n";
    ret += "}";
    return ret;
}
