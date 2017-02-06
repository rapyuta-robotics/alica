using namespace std;

#include "SystemConfig.h"

namespace supplementary
{
	// Initialize static variables
	string SystemConfig::rootPath;
	string SystemConfig::logPath;
	string SystemConfig::configPath;
	string SystemConfig::hostname;
	mutex SystemConfig::configsMapMutex;
	map<string, shared_ptr<Configuration> > SystemConfig::configs;

	/**
	 * The method for getting the singleton instance.
	 * @return A pointer to the SystemConfig object, you must not delete.
	 */
	SystemConfig* SystemConfig::getInstance()
	{
		static SystemConfig instance;
		return &instance;
	}

	/**
	 * The private constructor of the SystemConfig singleton.
	 */
	SystemConfig::SystemConfig()
	{
		// set the domain folder (1. by env-variable 2. by cwd)
		char *x = ::getenv(DOMAIN_FOLDER.c_str());
		if (x == NULL)
		{
			char cwd[4096];
			if (::getcwd(cwd, 4096) == NULL)
			{
				cerr << "SystemConfig: Error while calling getcwd!" << endl;
			}
			rootPath = cwd;
		}
		else
		{
			rootPath = x;
		}

		// set the domain config folger (1. by env-variable 2. by <domain folder>/etc
		x = ::getenv(DOMAIN_CONFIG_FOLDER.c_str());

		if (x == NULL)
		{
			configPath = (rootPath + "/etc/");
		}
		else
		{
			string temp = x;
			configPath = temp + "/";
		}

		logPath = FileSystem::combinePaths(rootPath, "/log/temp");
		if (!FileSystem::pathExists(logPath))
		{
			if (!FileSystem::createDirectory(logPath))
			{
				cerr << "SC: Could not create log directory: \"" << logPath << "\"" << endl;
			}
		}

		// set the hostname (1. by env-variable 2. by gethostname)
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

		cout << "SC: Root:           \"" << rootPath << "\"" << endl;
		cout << "SC: ConfigRoot:     \"" << configPath << "\"" << endl;
		cout << "SC: Hostname:       \"" << hostname << "\"" << endl;
		cout << "SC: Logging Folder: \"" << logPath << "\"" << endl;
	}

	void SystemConfig::shutdown()
	{

	}

	/**
	 * The access operator for choosing the configuration according to the given string
	 *
	 * @param s The string which determines the used configuration.
	 * @return The demanded configuration.
	 */
	Configuration* SystemConfig::operator[](const string s)
	{
		{
			lock_guard<mutex> lock(configsMapMutex);

			map<string, shared_ptr<Configuration> >::iterator itr = configs.find(s);

			if (itr != configs.end())
			{
				return itr->second.get();
			}
		}

		vector<string> files;

		string file = s + ".conf";

		// Check the local config
		//files.push_back(file);

		// Check the host-specific config
		string tempConfigPath = configPath;
		tempConfigPath = FileSystem::combinePaths(tempConfigPath, hostname);
		tempConfigPath = FileSystem::combinePaths(tempConfigPath, file);
		files.push_back(tempConfigPath);

		// Check the global config
		tempConfigPath = configPath;
		tempConfigPath = FileSystem::combinePaths(tempConfigPath, file);
		files.push_back(tempConfigPath);

		for (size_t i = 0; i < files.size(); i++)
		{
			if (FileSystem::pathExists(files[i]))
			{
				lock_guard<mutex> lock(configsMapMutex);

				//shared_ptr<Configuration> result = shared_ptr<Configuration>(new Configuration(files[i]));
				shared_ptr<Configuration> result = make_shared<Configuration>(files[i]);
				configs[s] = result;

				return result.get();
			}
		}

		// config-file not found, print error message
		cerr << "Configuration file " << file << " not found in either location:" << endl;
		for (size_t i = 0; i < files.size(); i++)
		{
			cerr << "- " << files[i] << endl;
		}
		return nullptr;
	}

	/**
	 * Looks up the own robot's ID with the system config's local hostname.
	 * @return The own robot's ID
	 */
	int SystemConfig::getOwnRobotID()
	{
		return SystemConfig::getRobotID(SystemConfig::getHostname());
	}

	/**
	 * Looks up the robot's ID with the given name.
	 * @return The robot's ID
	 */
	int SystemConfig::getRobotID(const string& name)
	{
		Configuration *tmp = (*SystemConfig::getInstance())["Globals"];
		int ownRobotID = tmp->get<int>("Globals", "Team", name.c_str(), "ID", NULL);
		return ownRobotID;
	}

	string SystemConfig::getRootPath()
	{
		return rootPath;
	}

	string SystemConfig::getConfigPath()
	{
		return configPath;
	}

	string SystemConfig::getLogPath()
	{
		return logPath;
	}

	string SystemConfig::getHostname()
	{
		return hostname;
	}

	void SystemConfig::setHostname(string newHostname)
	{
		hostname = newHostname;
		configs.clear();
		cout << "SC: Update Hostname:       \"" << hostname << "\"" << endl;
	}

	void SystemConfig::setRootPath(string rootPath)
	{
		this->rootPath = rootPath;
		cout << "SC: Update Root:           \"" << rootPath << "\"" << endl;
	}

	void SystemConfig::setConfigPath(string configPath)
	{
		this->configPath = configPath;
		cout << "SC: Update ConfigRoot:     \"" << configPath << "\"" << endl;
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

	string SystemConfig::robotNodeName(const string& nodeName)
	{
		return SystemConfig::getHostname() + NODE_NAME_SEPERATOR + nodeName;
	}

	string SystemConfig::getEnv(const string & var)
	{
		const char * val = ::getenv(var.c_str());
		if (val == 0)
		{
			cerr << "SC: Environment Variable " << var << " is null" << endl;
			return "";
		}
		else
		{
			cout << "SC: Environment Variable " << var << " is " << val << endl;
			return val;
		}
	}

}
