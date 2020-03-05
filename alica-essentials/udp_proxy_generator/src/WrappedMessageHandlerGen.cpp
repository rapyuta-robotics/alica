#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <vector>

#include "../include/WrappedMessage.h"
#include "boost/filesystem.hpp"
using boost::filesystem::exists;

using namespace std;

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

string getTemplateDir()
{
    string pwd = exec((string("rospack find udp_proxy_generator")).c_str());
    pwd.pop_back();
    return pwd + "/templates";
}

bool parseDefinitionFile(string msgDefFile, vector<WrappedMessage*>& msgList)
{
    string regstr = "(send|receive)Topic:\\s*(\\S+)\\s*WrappedMsg:\\s*(\\S+)\\s*Msg:\\s*(\\S+)\\s*Opt:\\s*\\[(.*)\\]";
    boost::regex line(regstr);
    std::ifstream ifs(msgDefFile);

    int i = 1;
    while (!ifs.eof()) {
        string s;
        std::getline(ifs, s);
        if ((s.length() > 0 && s[0] == '#') || s.length() < 1) {
            i++;
            continue;
        }
        if (boost::regex_match(s, line)) {
            boost::smatch m;
            string topic, wrappedMessage, message, options, sendReceive;
            if (boost::regex_search(s, m, line)) {
                if (m[1].compare("send") == 0 || m[1].compare("receive") == 0) {
                    sendReceive = m[1];
                    topic = m[2];
                    wrappedMessage = m[3];
                    message = m[4];
                    options = m[5];
                } else {
                    topic = m[1];
                    wrappedMessage = m[2];
                    message = m[3];
                    options = m[4];
                    sendReceive = "";
                }
            }

            WrappedMessage* msg = new WrappedMessage(topic, wrappedMessage, message, options, sendReceive);
            msgList.push_back(msg);
        } else {
            cout << "Parse Error in line " << i << " of " << msgDefFile << endl;
            cout << ">" << s << "<" << endl;
            return false;
        }
        i++;
    }
    return true;
}

string processTemplate(stringstream& t, vector<WrappedMessage*>& msgList)
{
    string reg_string = "<\\?(.*)\\?>";
    boost::regex markers(reg_string.c_str());
    boost::smatch m;
    stringstream ret;
    while (!t.eof()) {
        string s;
        std::getline(t, s);
        string matchType;
        if (boost::regex_search(s, m, markers)) {
            matchType = m[1];
        } else {
            ret << s << endl;
            continue;
        }
        s = trim(s);
        s = s.substr(2, s.length() - 4);

        if (s == "messageIncludes") {
            for (WrappedMessage* m : msgList) {
                ret << "#include \"" + m->message << ".h\"\n";
                ret << "#include \"" + m->wrappedMessage << ".h\"\n";
            }
        } else if (s == "subscriptions") {
            int i = 0;
            for (WrappedMessage* m : msgList) {
                if (m->sendReceive.compare("receive") == 0 || m->sendReceive.compare("") == 0) {
                    ret << "sub" << i++ << " = n.subscribe(\"/wrapped" << m->topic << "\"," << m->Ros2UdpQueueLength
                        << ", &WrappedMessageHandler::" << m->getRosWrappedCallBackName() << ",this);\n";
                }

                if (m->sendReceive.compare("send") == 0 || m->sendReceive.compare("") == 0) {
                    ret << "sub" << i++ << " = n.subscribe(\"" << m->topic << "\"," << m->Ros2UdpQueueLength
                        << ", &WrappedMessageHandler::" << m->getRosCallBackName() << ",this);\n";
                }
            }
        } else if (s == "rosMessageHandler") {
            for (WrappedMessage* m : msgList) {
                ret << m->getRosMessageHandler();
                ret << m->getRosWrappedMessageHandler();
            }
        } else if (s == "advertisement") {
            for (WrappedMessage* m : msgList) {
                ret << m->getWrappedPublisherName() << " = n.advertise<" << m->getWrappedRosClassName() << ">(\"/wrapped" << m->topic << "\","
                    << m->Udp2RosQueueLength << ",false);\n";
                ret << m->getPublisherName() << " = n.advertise<" << m->getRosClassName() << ">(\"" << m->topic << "\"," << m->Udp2RosQueueLength
                    << ",false);\n";
            }
        } else if (s == "rosPublisherDecl") {
            int i = 0;
            for (WrappedMessage* m : msgList) {
                ret << "ros::Publisher " << m->getPublisherName() << ";\n";
                ret << "ros::Publisher " << m->getWrappedPublisherName() << ";\n";
                ret << "ros::Subscriber sub" << i++ << ";\n";
                ret << "ros::Subscriber sub" << i++ << ";\n";
            }
        } else {
            cout << "Unknown Marker: " << s;
            exit(1);
        }
    }

    return ret.str();
}

vector<string> getFilesinFolder(string folder)
{
    namespace fs = boost::filesystem;
    fs::path someDir(folder.c_str());
    fs::directory_iterator end_iter;

    vector<string> result_set;

    if (exists(someDir) && is_directory(someDir)) {
        for (fs::directory_iterator dir_iter(someDir); dir_iter != end_iter; ++dir_iter) {
            if (fs::is_regular_file(dir_iter->status())) {
                result_set.push_back(string((*dir_iter).path().c_str()));
            }
        }
    }
    return result_set;
}

void processTemplates(string tmplDir, string outDir, vector<WrappedMessage*>& msgList)
{
    vector<string> tmplarr = getFilesinFolder(tmplDir); // = Directory.GetFiles(tmplDir,"*.*");
    for (string tmpl : tmplarr) {
        cout << "Template: " << tmpl << endl;
        int idx = tmpl.find_last_of('/');
        string basename = tmpl.substr(idx + 1);
        std::ifstream ifs(tmpl);
        stringstream ss;
        if (ifs) {
            ss << ifs.rdbuf();
        }
        ifs.close();
        if (basename.find("WrappedMessageHandler") == 0) {
            string parsedContent = processTemplate(ss, msgList);

            std::ofstream ofs(outDir + "/" + basename);
            ofs << parsedContent;
            ofs.close();
        }
    }
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        // TODO update this usage information
        cout << "Usage MakeUDPProxy.exe <packageName> <optional j>" << endl;
        return -1;
    }

    string outputPath;
    outputPath = exec((string("rospack find ") + argv[1]).c_str());
    cout << outputPath << endl;
    outputPath.pop_back();
    string templateDir = getTemplateDir();
    if (!exists(templateDir)) {
        cout << "Cannot find template directory: " << templateDir << endl;
    }

    if (!exists(outputPath)) {
        cout << "Cannot find package name!" << endl;
        return -1;
    }

    string wrappedMsgDefFile = outputPath + "/wrappedRelayMsgs.conf";
    outputPath = outputPath + "/include";

    if (!exists(wrappedMsgDefFile)) {
        cout << "Cannot find definition file " << wrappedMsgDefFile << endl;
        return -1;
    }
    vector<WrappedMessage*> msgList;
    if (!parseDefinitionFile(wrappedMsgDefFile, msgList)) {
        return -1;
    }

    bool reGenerate = true;

    if (reGenerate) {
        boost::filesystem::path dir(outputPath.c_str());
        if (!exists(outputPath) && !boost::filesystem::create_directories(dir)) {
            std::cout << "Failed to create Directory: " << outputPath << endl;
        }

        processTemplates(templateDir, outputPath, msgList);
    }
    for (auto d : msgList) {
        delete d;
    }
    cout << "Done" << endl;
    return 0;
}
