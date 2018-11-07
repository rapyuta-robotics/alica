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

const std::string DOMAIN_FOLDER = "DOMAIN_FOLDER";
const std::string DOMAIN_CONFIG_FOLDER = "DOMAIN_CONFIG_FOLDER";

namespace supplementary
{
class SystemConfig
{

  protected:
    static std::string rootPath;
    static std::string logPath;
    static std::string configPath;
    static std::string hostname;
    static std::mutex configsMapMutex;
    static std::map<std::string, std::shared_ptr<Configuration>> configs;
    static const char NODE_NAME_SEPERATOR = '_';

  public:
    static SystemConfig* getInstance();
    static void shutdown();
    static std::string robotNodeName(const std::string& nodeName);
    static int getOwnRobotID();
    static int getRobotID(const std::string& name);
    static std::string getHostname();
    static void setHostname(const std::string& newHostname);
    static void resetHostname();

    Configuration* operator[](const std::string& s);
    std::string getRootPath();
    std::string getConfigPath();
    std::string getLogPath();
    void setRootPath(std::string rootPath);
    void setConfigPath(std::string configPath);
    static std::string getEnv(const std::string& var);

  private:
    SystemConfig();
    ~SystemConfig(){};
};
} // namespace supplementary
