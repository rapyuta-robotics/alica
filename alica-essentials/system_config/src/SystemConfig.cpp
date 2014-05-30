using namespace std;

#include "SystemConfig.h"

namespace supplementary
{
	// Initialize static variables
	string SystemConfig::rootPath;
	string SystemConfig::configPath;
	string SystemConfig::hostname;
	mutex SystemConfig::configsMapMutex;
	map<string, shared_ptr<Configuration> > SystemConfig::configs;

	/**
	 * The method for getting the singlton instance.
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

		/*cout << "Root:       " << rootPath << endl;
		cout << "ConfigRoot: " << configPath << endl;
		cout << "Hostname:   " << hostname << endl;*/
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
		files.push_back(file);

		// Check the host-specific config
		string tempConfigPath = configPath;
		tempConfigPath = tempConfigPath + "/" + hostname;
		tempConfigPath = tempConfigPath + "/" + file;
		files.push_back(tempConfigPath);

		// Check the global config
		tempConfigPath = configPath;
		tempConfigPath = tempConfigPath + "/" + file;
		files.push_back(tempConfigPath);

		for (size_t i = 0; i < files.size(); i++)
		{
			if (fileExists(files[i]))
			{
				lock_guard<mutex> lock(configsMapMutex);

				shared_ptr<Configuration> result = shared_ptr<Configuration>(new Configuration(files[i]));
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

//	int SystemConfig::GetOwnRobotID()
//	{
//		if (ownRobotID != 0)
//			return ownRobotID;
//
//		Configuration *tmp = (*SystemConfig::getInstance())["Globals"];
//		ownRobotID = tmp->get<int>("Globals", "Team", SystemConfig::getHostname().c_str(), "ID", NULL);
//		/* Philipp Sperber:
//		 * Never delete a Configuration Pointer!!!
//		 * Messes up on next call
//		 */
//		delete tmp;
//		return ownRobotID;
//	}

	string SystemConfig::getRootPath()
	{
		return rootPath;
	}

	string SystemConfig::getConfigPath()
	{
		return configPath;
	}

	string SystemConfig::getHostname()
	{
		return hostname;
	}

	void SystemConfig::setHostname(string newHostname)
	{
		hostname = newHostname;
		configs.clear();
	}

	void SystemConfig::setRootPath(string rootPath)
	{
		this->rootPath = rootPath;
	}

	void SystemConfig::setConfigPath(string configPath)
	{
		this->configPath = configPath;
	}

//	void SystemConfig::resetHostname()
//	{
//		char* envname = ::getenv("ROBOT");
//		if ((envname == NULL) || ((*envname) == 0x0))
//		{
//			char hn[1024];
//			hn[1023] = '\0';
//			gethostname(hn, 1023);
//			SystemConfig::hostname = hn;
//		}
//		else
//		{
//			hostname = envname;
//		}
//		configs.clear();
//	}

//	string SystemConfig::robotNodeName(const string& nodeName)
//	{
//		return SystemConfig::getHostname() + NODE_NAME_SEPERATOR + nodeName;
//	}

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

	/**
	 * Checks whether a given file exists.
	 * @param filename Absolute path to file.
	 * @return true if the file exists, false otherwise.
	 */
	bool SystemConfig::fileExists(const string& filename)
	{
		struct stat buf;
		if (stat(filename.c_str(), &buf) != -1)
		{
			return true;
		}
		return false;
	}
}
