using namespace std;

#include "SystemConfig.h"

namespace supplementary
{
// Initialize static variables
	bool SystemConfig::initialized = false;

	mutex SystemConfig::scMutex;

	atomic<SystemConfigPtr> SystemConfig::instance;

	string SystemConfig::rootPath;
	string SystemConfig::configPath;
	int SystemConfig::ownRobotID = 0;

	std::string SystemConfig::hostname;

	std::map<std::string, std::shared_ptr<Configuration> > SystemConfig::configs;

	SystemConfigPtr SystemConfig::getInstance()
	{
		// 100% thread safe double check singlton variant
		// see: http://preshing.com/20130930/double-checked-locking-is-fixed-in-cpp11/
		SystemConfigPtr tmp = atomic_load_explicit(&instance,std::memory_order_acquire);
		//SystemConfigPtr tmp = instance.load(memory_order_relaxed);
		atomic_thread_fence(memory_order_acquire);
		if (tmp == nullptr)
		{
			lock_guard<mutex> lock(scMutex);
			tmp = instance.load(memory_order_relaxed);
			if (tmp == nullptr)
			{
				tmp = shared_ptr<SystemConfig>(new SystemConfig());
				atomic_thread_fence(memory_order_release);
				//instance.store(tmp, memory_order_relaxed);
				atomic_store_explicit(&instance,tmp,memory_order_release);
			}
		}
		return tmp;
	}

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
			std::string temp = x;
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

		std::cout << "Root:       " << rootPath << std::endl;
		std::cout << "ConfigRoot: " << configPath << std::endl;
		std::cout << "Hostname:   " << hostname << std::endl;

	}

	Configuration *SystemConfig::operator[](const std::string s)
	{

		{
			std::lock_guard<std::mutex> lock(mutex);

			std::map<std::string, std::shared_ptr<Configuration> >::iterator itr = configs.find(s);

			if (itr != configs.end())
			{
				return itr->second.get();
			}
		}

		std::vector<std::string> files;

		std::string file = s + ".conf";

		// Check the local config
		files.push_back(file);
		std::string tempConfigPath = configPath;

		tempConfigPath = tempConfigPath + "/" + hostname;

		tempConfigPath = tempConfigPath + "/" + file;
		// Check the host-specific config
		files.push_back(tempConfigPath);

		tempConfigPath = configPath;
		tempConfigPath = tempConfigPath + "/" + file;

		// Check the global config
		files.push_back(tempConfigPath);

		for (size_t i = 0; i < files.size(); i++)
		{
			if (fileExists(files[i]))
			{

				std::lock_guard<std::mutex> lock(mutex);

				std::shared_ptr<Configuration> result = std::shared_ptr<Configuration>(new Configuration(files[i]));
				configs[s] = result;

				return result.get();
			}
		}

		std::ostringstream ss;

		ss << "Configuration file " << file << " not found in either location:" << std::endl;

		for (size_t i = 0; i < files.size(); i++)
		{
			ss << "- " << files[i] << std::endl;
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
		return rootPath;
	}

	std::string SystemConfig::getConfigPath()
	{
		return configPath;
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

	void SystemConfig::setRootPath(std::string rootPath)
	{
		this->rootPath = rootPath;
	}

	void SystemConfig::setConfigPath(std::string configPath)
	{
		this->configPath = configPath;
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
	bool SystemConfig::fileExists(const std::string& filename)
	{
		struct stat buf;
		if (stat(filename.c_str(), &buf) != -1)
		{
			return true;
		}
		return false;
	}
}
