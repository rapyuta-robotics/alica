#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

#include <boost/thread.hpp>

#include "SystemConfig.h"

// Initialize static variables
bool SystemConfig::initialized = false;

boost::mutex SystemConfig::mutex;

SystemConfigPtr SystemConfig::instance;

boost::filesystem::path SystemConfig::rootPath;
boost::filesystem::path SystemConfig::libPath;
boost::filesystem::path SystemConfig::logPath;
boost::filesystem::path SystemConfig::configPath;
int SystemConfig::ownRobotID;

std::string SystemConfig::hostname;

std::map<std::string, boost::shared_ptr<Configuration> > SystemConfig::configs;

SystemConfigPtr SystemConfig::getInstance()
{

  boost::mutex::scoped_lock lock(mutex);

  if (!initialized)
  {

    instance = boost::shared_ptr<SystemConfig>(new SystemConfig());

    char *x = ::getenv(DOMAIN_FOLDER.c_str());

    if (x == NULL)
    {

      char cwd[PATH_MAX];
      if (::getcwd(cwd, PATH_MAX) == NULL)
      {
        std::cout << "SystemConfig: Error while calling getcwd!" << std::endl;
      }

      rootPath = boost::filesystem::path(cwd).normalize();

    }
    else
    {
      rootPath = boost::filesystem::path(x).normalize();
    }

    x = ::getenv("ES_CONFIG_ROOT");

    if (x == NULL)
    {
      configPath = (rootPath / "etc").normalize();
    }
    else
    {
      configPath = boost::filesystem::path(x).normalize();
    }

    libPath = (rootPath / "lib").normalize();
    logPath = (rootPath / "log").normalize();
    char* envname = ::getenv("ROBOT");
    if ((envname == NULL) || ((*envname) == 0x0))
    {
      char hn[1024];
      hn[1023] = '\0';
      gethostname(hn, 1023);
      SystemConfig::hostname = hn;
    }
    else
    {
      hostname = envname;
    }
    std::cout << "Root:       " << rootPath << std::endl;
    std::cout << "ConfigRoot: " << configPath << std::endl;
    std::cout << "LibRoot:    " << libPath << std::endl;
    std::cout << "LogRoot:    " << logPath << std::endl;
    std::cout << "Hostname:   " << hostname << std::endl;

    initialized = true;
  }

  return instance;
}

Configuration *SystemConfig::operator[](const std::string s)
{

  {
    boost::mutex::scoped_lock lock(mutex);

    std::map<std::string, boost::shared_ptr<Configuration> >::iterator itr = configs.find(s);

    if (itr != configs.end())
    {
      return itr->second.get();
    }
  }

  std::vector<boost::filesystem::path> files;

  std::string file = s + ".conf";

  // Check the local config
  files.push_back(boost::filesystem::path(file));

  boost::filesystem::path path(configPath);
  path /= hostname;
  path /= file;

  // Check the host-specific config
  files.push_back(path);

  path = boost::filesystem::path(configPath);
  path /= file;

  // Check the global config
  files.push_back(path);

  for (size_t i = 0; i < files.size(); i++)
  {

    if (boost::filesystem::exists(files[i]))
    {

      boost::mutex::scoped_lock lock(mutex);

      boost::shared_ptr<Configuration> result = boost::shared_ptr<Configuration>(new Configuration(files[i].string()));
      configs[s] = result;

      return result.get();
    }
  }

  std::ostringstream ss;

  ss << "Configuration file " << file << " not found in either location:" << std::endl;

  for (size_t i = 0; i < files.size(); i++)
  {
    ss << "- " << files[i].string() << std::endl;
  }

  throw ConfigException(ss.str());
}
int SystemConfig::GetOwnRobotID()
{
  if (ownRobotID != 0)
    return ownRobotID;
  Configuration *tmp = (*SystemConfig::getInstance())["Globals"];
  ownRobotID = tmp->get<int>("Globals", "Team", SystemConfig::getHostname().c_str(), "ID", NULL);
  /* Philipp Sperber:
   * Never delete a Configuration Pointer!!!
   * Messes up on next call
   */
//		delete tmp;
  return ownRobotID;
}
std::string SystemConfig::getRootPath()
{
  return rootPath.string();
}

std::string SystemConfig::getLibPath()
{
  return libPath.string();
}

std::string SystemConfig::getLogPath()
{
  return logPath.string();
}

std::string SystemConfig::getConfigPath()
{
  return configPath.string();
}

std::string SystemConfig::getHostname()
{
  return hostname;
}

void SystemConfig::setHostname(std::string newHostname)
{
  hostname = newHostname;
  configs.clear();
}

void SystemConfig::resetHostname()
{
  char* envname = ::getenv("ROBOT");
  if ((envname == NULL) || ((*envname) == 0x0))
  {
    char hn[1024];
          hn[1023] = '\0';
          gethostname(hn, 1023);
          SystemConfig::hostname = hn;
  }
  else
  {
    hostname = envname;
  }
  configs.clear();
}

std::string SystemConfig::robotNodeName(const std::string& nodeName)
{
  return SystemConfig::getHostname() + NODE_NAME_SEPERATOR + nodeName;
}

std::string SystemConfig::GetEnv(const std::string & var)
{
  const char * val = ::getenv(var.c_str());
  if (val == 0)
  {
    std::cerr << "Environment Variable " << var << " is null" << std::endl;
    return "";
  }
  else
  {
    std::cout << "Environment Variable " << var << " is " << val << std::endl;
    return val;
  }
}
