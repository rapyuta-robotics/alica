#ifndef SYSTEMCONFIG_H_
#define SYSTEMCONFIG_H_

#include <map>
#include <string>
#include <mutex>
#include <memory>

#include <boost/thread/mutex.hpp>
#include <boost/filesystem.hpp>

#include "Configuration.h"

const std::string DOMAIN_FOLDER="DOMAIN_FOLDER";

class SystemConfig;

typedef std::shared_ptr<SystemConfig> SystemConfigPtr;

class SystemConfig
{

protected:

  SystemConfig()
  {
    ownRobotID = 0;
  }

  static bool initialized;

  static std::mutex mutex;

  static SystemConfigPtr instance;

  static std::string rootPath;
  static std::string libPath;
  static std::string logPath;
  static std::string configPath;

  static std::string hostname;

  static std::map<std::string, std::shared_ptr<Configuration> > configs;

  static const char NODE_NAME_SEPERATOR = '_';
  static int ownRobotID;

public:

  static SystemConfigPtr getInstance();

  static std::string robotNodeName(const std::string& nodeName);

  static int GetOwnRobotID();
  static std::string getHostname();
  static void setHostname(std::string newHostname);
  static void resetHostname();

  Configuration *operator[](const std::string s);

  std::string getRootPath();
  std::string getLibPath();
  std::string getLogPath();
  std::string getConfigPath();
  bool fileExists(const std::string& filename);
  static std::string GetEnv(const std::string& var);
};

#endif /* SYSTEMCONFIG_H_ */
