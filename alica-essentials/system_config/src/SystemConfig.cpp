#include "essentials/SystemConfig.h"

#include "essentials/Configuration.h"

#include <unistd.h>

namespace essentials
{
using std::cerr;
using std::cout;
using std::endl;
using std::mutex;
using std::shared_ptr;
using std::string;
using std::vector;

/**
 * The method for getting the singleton instance.
 * @return A pointer to the SystemConfig object, you must not delete.
 */
SystemConfig& SystemConfig::getInstance()
{
    static SystemConfig instance;
    return instance;
}

/**
 * The private constructor of the SystemConfig singleton.
 */
SystemConfig::SystemConfig()
{
    // set the domain folder (1. by env-variable 2. by cwd)
    char* x = ::getenv(DOMAIN_FOLDER.c_str());
    if (x == NULL) {
        char cwd[4096];
        if (::getcwd(cwd, 4096) == NULL) {
            cerr << "SystemConfig: Error while calling getcwd!" << endl;
        }
        rootPath = cwd;
    } else {
        rootPath = x;
    }

    // set the domain config folder (1. by env-variable 2. by <domain folder>/etc
    x = ::getenv(DOMAIN_CONFIG_FOLDER.c_str());

    if (x == NULL) {
        configPath = FileSystem::combinePaths(rootPath, "/etc");
    } else {
        configPath = x;
    }
    if (!FileSystem::pathExists(configPath)) {
        cerr << "SC: Could not find config directory: \"" << configPath << "\"" << endl;
    }
    if (!essentials::FileSystem::endsWith(configPath, essentials::FileSystem::PATH_SEPARATOR)) {
        configPath = configPath + essentials::FileSystem::PATH_SEPARATOR;
    }

    logPath = FileSystem::combinePaths(rootPath, "/log/temp");
    if (!FileSystem::pathExists(logPath)) {
        if (!FileSystem::createDirectory(logPath)) {
            cerr << "SC: Could not create log directory: \"" << logPath << "\"" << endl;
        }
    }

    // set the hostname (1. by env-variable 2. by gethostname)
    char* envname = ::getenv("ROBOT");
    if ((envname == NULL) || ((*envname) == 0x0)) {
        char hn[1024];
        hn[1023] = '\0';
        gethostname(hn, 1023);
        SystemConfig::hostname = hn;
    } else {
        hostname = envname;
    }

    cout << "SC: Root:           \"" << rootPath << "\"" << endl;
    cout << "SC: ConfigRoot:     \"" << configPath << "\"" << endl;
    cout << "SC: Hostname:       \"" << hostname << "\"" << endl;
    cout << "SC: Logging Folder: \"" << logPath << "\"" << endl;
}

void SystemConfig::shutdown() {}

/**
 * The access operator for choosing the configuration according to the given string
 *
 * @param s The string which determines the used configuration.
 * @return The demanded configuration.
 */
Configuration* SystemConfig::operator[](const std::string& s)
{
    {
        std::lock_guard<mutex> lock(configsMapMutex);

        std::map<std::string, std::shared_ptr<Configuration>>::iterator itr = configs.find(s);

        if (itr != configs.end()) {
            return itr->second.get();
        }
    }

    std::string file_name = getConfigFileName(s);
    if (file_name.empty()) {
        return nullptr;
    } else {
        std::lock_guard<mutex> lock(configsMapMutex);
        std::shared_ptr<Configuration> result = std::make_shared<Configuration>(file_name);
        configs[s] = result;

        return result.get();
    }
}

std::string SystemConfig::getConfigFileName(const std::string& s) {
    string file = s + ".conf";
    // Check the host-specific config
    vector<string> files;
    string tempConfigPath = configPath;
    tempConfigPath = FileSystem::combinePaths(tempConfigPath, hostname);
    tempConfigPath = FileSystem::combinePaths(tempConfigPath, file);
    files.push_back(tempConfigPath);

    // Check the global config
    tempConfigPath = configPath;
    tempConfigPath = FileSystem::combinePaths(tempConfigPath, file);
    files.push_back(tempConfigPath);

    for (size_t i = 0; i < files.size(); i++) {
        if (FileSystem::pathExists(files[i])) {
            return files[i];
        }
    }

    // config-file not found, print error message
    cerr << "Configuration file " << file << " not found in either location:" << endl;
    for (size_t i = 0; i < files.size(); i++) {
        cerr << "- " << files[i] << endl;
    }
    return "";
}

/**
 * Looks up the own robot's ID with the system config's local hostname.
 * @return The own robot's ID
 * <deprecated>
 */
int SystemConfig::getOwnRobotID()
{
    return SystemConfig::getRobotID(SystemConfig::getHostname());
}

/**
 * Looks up the robot's ID with the given name.
 * @return The robot's ID
 * <deprecated>
 */
int SystemConfig::getRobotID(const string& name)
{
    // TODO this should be optional for dynamic teams (is it ok to return ints?)
    Configuration* tmp = (SystemConfig::getInstance())["Globals"];
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

void SystemConfig::setHostname(const std::string& newHostname)
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
    if ((envname == NULL) || ((*envname) == 0x0)) {
        char hn[1024];
        hn[1023] = '\0';
        gethostname(hn, 1023);
        SystemConfig::hostname = hn;
    } else {
        hostname = envname;
    }
    configs.clear();
}

string SystemConfig::robotNodeName(const string& nodeName)
{
    return SystemConfig::getHostname() + NODE_NAME_SEPERATOR + nodeName;
}

string SystemConfig::getEnv(const string& var)
{
    const char* val = ::getenv(var.c_str());
    if (val == 0) {
        cerr << "SC: Environment Variable " << var << " is null" << endl;
        return "";
    } else {
        cout << "SC: Environment Variable " << var << " is " << val << endl;
        return val;
    }
}
} // namespace supplementary
