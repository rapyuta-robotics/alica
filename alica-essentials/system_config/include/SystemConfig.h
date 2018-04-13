#pragma once

#include "Configuration.h"

#include <FileSystem.h>

#include <atomic>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

const string DOMAIN_FOLDER = "DOMAIN_FOLDER";
const string DOMAIN_CONFIG_FOLDER = "DOMAIN_CONFIG_FOLDER";

namespace supplementary
{
class SystemConfig
{

  protected:
    static string rootPath;
    static string logPath;
    static string configPath;
    static string hostname;
    static std::mutex configsMapMutex;
    static std::map<std::string, std::shared_ptr<Configuration>> configs;
    static const char NODE_NAME_SEPERATOR = '_';

  public:
    static SystemConfig *getInstance();
    static void shutdown();
    static string robotNodeName(const string &nodeName);
    static int getOwnRobotID();
    static int getRobotID(const string &name);
    static string getHostname();
    static void setHostname(string newHostname);
    static void resetHostname();

    Configuration *operator[](const string s);
    string getRootPath();
    string getConfigPath();
    string getLogPath();
    void setRootPath(string rootPath);
    void setConfigPath(string configPath);
    static string getEnv(const string &var);

  private:
    SystemConfig();
    ~SystemConfig(){};
};
}
