#ifndef SYSTEMCONFIG_H_
#define SYSTEMCONFIG_H_

using namespace std;

#include <map>
#include <string>
#include <mutex>
#include <thread>
#include <memory>
#include <atomic>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>

#include "Configuration.h"

const string DOMAIN_FOLDER = "DOMAIN_FOLDER";
const string DOMAIN_CONFIG_FOLDER = "DOMAIN_CONFIG_FOLDER";

namespace supplementary
{
	class SystemConfig
	{

	protected:
		static string rootPath;
		static string configPath;
		static string hostname;
		static mutex configsMapMutex;
		static map<string, shared_ptr<Configuration> > configs;
//		static const char NODE_NAME_SEPERATOR = '_';

	public:
		static SystemConfig* getInstance();
		static string robotNodeName(const string& nodeName);
		static int GetOwnRobotID();
		static string getHostname();
		static void setHostname(string newHostname);
		//static void resetHostname();
		Configuration *operator[](const string s);
		string getRootPath();
		string getConfigPath();bool fileExists(const string& filename);
		void setRootPath(string rootPath);
		void setConfigPath(string configPath);
		static string getEnv(const string& var);

	private:
		SystemConfig();
		~SystemConfig(){};
	};
}
#endif /* SYSTEMCONFIG_H_ */
