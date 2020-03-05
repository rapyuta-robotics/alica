#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <vector>

#include "boost/filesystem.hpp"

#include "../include/RelayedMessage.h"
#include "RelayedMessage.h"

using boost::filesystem::exists;

// trim from start
inline std::string& ltrim(std::string& s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

// trim from end
inline std::string& rtrim(std::string& s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

// trim from both ends
inline std::string& trim(std::string& s)
{
    return ltrim(rtrim(s));
}

std::string exec(const char* cmd)
{
    FILE* pipe = popen(cmd, "r");
    if (!pipe)
        return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}

std::string getTemplateDir()
{
    std::string pwd = exec((std::string("rospack find udp_proxy_generator")).c_str());
    pwd.pop_back();
    return pwd + "/templates";
}

bool checkForCollisions(std::vector<RelayedMessage*>& msgList)
{
    for (int i = 0; i < msgList.size(); i++) {
        for (int j = i + 1; j < msgList.size(); j++) {
            if (msgList[i]->Id == msgList[j]->Id) {
                std::cout << "Hashcollision between topic " << msgList[i]->Topic << " and topic " << msgList[j]->Topic;
                return false;
            }
        }
    }
    return true;
}

bool parseDefinitionFile(std::string msgDefFile, std::vector<RelayedMessage*>& msgList, std::string& lang)
{
    // regex line("Topic:\\^ ");
    // regex line("Topic:\\.*(\\.+)\\.*Msg:\\.*(\\.+)\\.*Opt:\\.*\\[(.*)\\]");
    std::string regstr;
    if (lang.compare("java") == 0) {
        regstr = "(send|receive)Topic:\\s*(\\S+)\\s*Msg:\\s*(\\S+)\\s*Opt:\\s*\\[(.*)\\]";
    } else {
        regstr = "Topic:\\s*(\\S+)\\s*Msg:\\s*(\\S+)\\s*Opt:\\s*\\[(.*)\\]";
    }
    boost::regex line(regstr);
    // regex line("Topic:\s*(^ )\s*Msg:\s*(^ )\s*Opt:\s*\[(.*)\]");
    std::ifstream ifs(msgDefFile);

    int i = 1;
    while (!ifs.eof()) {
        std::string s;
        std::getline(ifs, s);
        if ((s.length() > 0 && s[0] == '#') || s.length() < 1) {
            i++;
            continue;
        }
        if (boost::regex_match(s, line)) {
            boost::smatch m;
            std::string topic, message, options, sendReceive;
            if (boost::regex_search(s, m, line)) {
                if (m[1].compare("send") == 0 || m[1].compare("receive") == 0) {
                    sendReceive = m[1];
                    topic = m[2];
                    message = m[3];
                    options = m[4];
                } else {
                    topic = m[1];
                    message = m[2];
                    options = m[3];
                    sendReceive = "";
                }
                // for (int n=1; n<m.size(); n++)
                //	std::cout << m[n] << " " << n << " ";
                // std::cout << std::endl;
                // s = m.suffix().str();
            }

            // Match m = line.Match(s);
            RelayedMessage* msg = new RelayedMessage(topic, message, options, sendReceive);
            msgList.push_back(msg);
        } else {
            std::cout << "Parse Error in line " << i << " of " << msgDefFile << std::endl;
            std::cout << ">" << s << "<" << std::endl;
            return false;
        }
        i++;
    }
    return true;
}

std::string processTemplate(std::stringstream& t, std::vector<RelayedMessage*>& msgList, std::string& pkgName)
{
    std::string reg_string = "<\\?(.*)\\?>";
    boost::regex markers(reg_string.c_str());
    boost::smatch m;
    std::stringstream ret;
    while (!t.eof()) {
        std::string s;
        std::getline(t, s);
        std::string matchType;
        if (boost::regex_search(s, m, markers)) {
            matchType = m[1];
        } else {
            ret << s << std::endl;
            continue;
        }
        s = trim(s);
        s = s.substr(2, s.length() - 4);

        if (s == "messageIncludes") {
            for (RelayedMessage* m : msgList) {
                ret << "#include \"" + m->FullName << ".h\"\n";
            }
        } else if (s == "subscriptions") {
            int i = 0;
            for (RelayedMessage* m : msgList) {
                if (m->UseRosTcp) {
                    ret << "ros::Subscriber sub" << i << " = n.subscribe(\"" << m->Topic << "\"," << m->Ros2UdpQueueLength << ", " << m->getRosCallBackName()
                        << ");\n";
                } else {
                    ret << "ros::Subscriber sub" << i << " = n.subscribe(\"" << m->Topic << "\"," << m->Ros2UdpQueueLength << ", " << m->getRosCallBackName()
                        << ",ros::TransportHints().unreliable().tcpNoDelay().reliable());\n";
                }
                i++;
            }
        } else if (s == "rosMessageHandler") {
            for (RelayedMessage* m : msgList) {
                ret << m->getRosMessageHandler();
            }
        } else if (s == "advertisement") {
            for (RelayedMessage* m : msgList) {
                ret << m->getPublisherName() << " = n.advertise<" << m->getRosClassName() << ">(\"" << m->Topic << "\"," << m->Udp2RosQueueLength
                    << ",false);\n";
            }
        } else if (s == "rosPublisherDecl") {
            for (RelayedMessage* m : msgList) {
                ret << "ros::Publisher " << m->getPublisherName() << ";\n";
            }
        } else if (s == "udpReception") {
            for (RelayedMessage* m : msgList) {
                ret << "case " << m->Id << "ul: {\n";
                ret << m->getRosClassName() << " m" << m->Id << ";\n";
                ret << "ros::serialization::Serializer<" << m->getRosClassName() << ">::read(stream, m" << m->Id << ");\n";
                ret << m->getPublisherName() << ".publish<" << m->getRosClassName() << ">(m" << m->Id << ");\n";
                ret << "break; }\n";
            }
        } else if (s == "configfile") {
            ret << "Configuration *proxyconf = (*sc)[\"" << pkgName << "\"];";
        } else if (s == "nodename") {
            ret << "ros::init(argc, argv, \"" << pkgName << "\");";
        } else {
            std::cout << "Unknown Marker: " << s;
            exit(1);
        }
    }

    return ret.str();
}

std::string processTemplateJava(std::stringstream& t, std::vector<RelayedMessage*>& msgList, std::string& pkgName)
{
    std::string reg_string = "<\\?(.*)\\?>";
    boost::regex markers(reg_string.c_str());
    boost::smatch m;
    std::stringstream ret;
    while (!t.eof()) {
        std::string s;
        std::getline(t, s);
        std::string matchType;
        if (boost::regex_search(s, m, markers)) {
            matchType = m[1];
        } else {
            ret << s << std::endl;
            continue;
        }
        s = trim(s);
        s = s.substr(2, s.length() - 4);

        if (s == "messageIncludes") {
            for (RelayedMessage* m : msgList) {
                ret << "import " + m->FullNameJava << ";\n";
            }
        } else if (s == "subscriptions") {
            int i = 0;
            for (RelayedMessage* m : msgList) {
                if (m->SendReceiveString.compare("send") == 0 || m->SendReceiveString.compare("") == 0) {
                    ret << "final Subscriber sub" << i << " = connectedNode.newSubscriber(\"" << m->Topic << "\", \"" << m->FullName << "\");\n";
                    ret << "sub" << i << ".addMessageListener(new " << m->getRosJavaCallBackName() << "());\n";
                    i++;
                }
            }
        } else if (s == "rosMessageHandler") {
            for (RelayedMessage* m : msgList) {
                if (m->SendReceiveString.compare("send") == 0 || m->SendReceiveString.compare("") == 0) {
                    ret << m->getRosJavaMessageHandler();
                }
            }
        } else if (s == "advertisement") {
            for (RelayedMessage* m : msgList) {
                ret << m->getPublisherName() << " = connectedNode.newPublisher(\"" << m->Topic << "\", \"" << m->FullName << "\");\n";
            }
        } else if (s == "rosPublisherDecl") {
            for (RelayedMessage* m : msgList) {
                ret << "private Publisher<" << m->BaseName << "> " << m->getPublisherName() << ";\n";
            }
        } else if (s == "udpReception") {
            bool first = true;
            for (RelayedMessage* m : msgList) {
                if (m->SendReceiveString.compare("receive") == 0 || m->SendReceiveString.compare("") == 0) {
                    if (!first) {
                        ret << "else ";
                    } else {
                        first = false;
                    }
                    ret << "if(id == " << m->Id << "l) {\n";
                    ret << "MessageDeserializer<" + m->BaseName + "> deserializer = node.getMessageSerializationFactory().newMessageDeserializer("
                        << m->BaseName << "._TYPE);\n";
                    ret << "byte[] message = Arrays.copyOfRange(packet.getData(), Integer.SIZE / Byte.SIZE, "
                           "packet.getData().length-4);\n";
                    ret << m->BaseName << " m" << m->Id
                        << " = "
                           "deserializer.deserialize(ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN,message));\n";
                    ret << m->getPublisherName() << ".publish(m" << m->Id << ");\n";
                    ret << "}\n";
                }
            }
        } else if (s == "configfile") {
            boost::algorithm::to_lower(pkgName);
            ret << "udpConfig.load(getActivity().getResources().openRawResource(R.raw." << pkgName << "));\n";
        } else if (s == "nodename") {
            // The proxy starts as part of the application in java and therefore needs no explicit start call
        } else {
            std::cout << "Unknown Marker: " << s;
            exit(1);
        }
    }

    return ret.str();
}

std::vector<std::string> getFilesinFolder(std::string folder)
{
    namespace fs = boost::filesystem;
    fs::path someDir(folder.c_str());
    fs::directory_iterator end_iter;

    std::vector<std::string> result_set;

    if (exists(someDir) && is_directory(someDir)) {
        for (fs::directory_iterator dir_iter(someDir); dir_iter != end_iter; ++dir_iter) {
            if (fs::is_regular_file(dir_iter->status())) {
                result_set.push_back(std::string((*dir_iter).path().c_str()));
            }
        }
    }
    return result_set;
}

void processTemplates(std::string tmplDir, std::string outDir, std::vector<RelayedMessage*>& msgList, std::string& pkgName)
{
    std::vector<std::string> tmplarr = getFilesinFolder(tmplDir); // = Directory.GetFiles(tmplDir,"*.*");
    for (std::string tmpl : tmplarr) {
        std::cout << "Template: " << tmpl << std::endl;
        int idx = tmpl.find_last_of('/');
        std::string basename = tmpl.substr(idx + 1);
        std::ifstream ifs(tmpl);
        std::stringstream ss;
        if (ifs) {
            ss << ifs.rdbuf();
        }
        ifs.close();

        std::string content = ss.str();
        if (basename.find(".tpl") != std::string::npos && outDir.find("proxy_gen") != std::string::npos) {
            std::string parsedContent = processTemplate(ss, msgList, pkgName);

            std::string outputFile = outDir + "/" + basename.substr(0, basename.length() - 3) + "cpp";
            std::ofstream ofs(outputFile);
            ofs << parsedContent;
            ofs.close();

        } else if (outDir.find("src/main/java/util") != std::string::npos) {
            std::string parsedContent = processTemplateJava(ss, msgList, pkgName);

            std::string outputFile = outDir + "/" + basename.substr(0, basename.length() - 4) + "java";
            std::ofstream ofs(outputFile);
            ofs << parsedContent;
            ofs.close();
        }
    }
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        // TODO update this usage information
        std::cout << "Usage MakeUDPProxy.exe <packageName> <optional j>" << std::endl;
        return -1;
    }

    std::vector<std::string> outputPaths;
    std::string outputPath;
    outputPath = exec((std::string("rospack find ") + argv[1]).c_str());
    std::cout << outputPath << std::endl;
    outputPath.pop_back();
    std::string templateDir = getTemplateDir();
    if (!exists(templateDir)) {
        std::cout << "Cannot find template directory: " << templateDir << std::endl;
    }

    std::string pkgName = argv[1];

    if (!exists(outputPath)) {
        std::cout << "Cannot find package name!" << std::endl;
        return -1;
    }
    std::string msgDefFile = outputPath + "/relayMsgs.conf";
    std::string lang;
    if (argc == 3 && argv[2][0] == 'j') {
        if (outputPath.find_last_of('/') != std::string::npos) {
            outputPath += outputPath.substr(outputPath.find_last_of('/')) + "/src/main/java/util";
        }
        lang = "java";
    } else {
        lang = "cpp";
        outputPath = outputPath + "/proxy_gen";
    }

    if (!exists(msgDefFile)) {
        std::cout << "Cannot find definition file " << msgDefFile << std::endl;
        return -1;
    }
    std::vector<RelayedMessage*> msgList;
    if (!parseDefinitionFile(msgDefFile, msgList, lang)) {
        return -1;
    }
    if (!checkForCollisions(msgList)) {
        return -1;
    }

    bool reGenerate = true;
    if (reGenerate) {
        boost::filesystem::path dir(outputPath.c_str());
        if (!exists(outputPath) && !boost::filesystem::create_directories(dir)) {
            std::cout << "Failed to create Directory: " << outputPath << std::endl;
        }
        processTemplates(templateDir, outputPath, msgList, pkgName);
    }
    for (auto d : msgList) {
        delete d;
    }
    std::cout << "Done" << std::endl;
    return 0;
}
