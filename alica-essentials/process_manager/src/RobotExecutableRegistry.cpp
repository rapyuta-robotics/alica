#include "process_manager/ExecutableMetaData.h"
#include "process_manager/RobotMetaData.h"

#include <ConsoleCommandHelper.h>
#include <SystemConfig.h>
#include <iostream>
#include <msl/robot/IntRobotIDFactory.h>
#include <process_manager/RobotExecutableRegistry.h>
#include <supplementary/IAgentIDFactory.h>

using std::string;

namespace supplementary
{

RobotExecutableRegistry *RobotExecutableRegistry::get()
{
    static RobotExecutableRegistry instance;
    return &instance;
}

RobotExecutableRegistry::RobotExecutableRegistry()
    : sc(SystemConfig::getInstance())
    , agentIDFactory(nullptr)
{
    string idType = (*sc)["ProcessManaging"]->get<string>("ProcessManager.agentIDType", NULL);
    if (idType.compare("int") == 0)
    {
        this->agentIDFactory = new ::msl::robot::IntRobotIDFactory();
    }
    else if (idType.compare("uuid") == 0)
    {
        // TODO
        throw new runtime_error("RobotExecutableRegistry: UUID Factory needs to be implemented!");
    }
    else
    {
        throw new runtime_error("RobotExecutableRegistry: Unknown Agent ID Type in ProcessManaging.conf: '" + idType +
                                "'");
    }
}

RobotExecutableRegistry::~RobotExecutableRegistry()
{
    for (auto metaData : this->executableList)
    {
        delete metaData;
    }

    for (auto metaData : this->robotList)
    {
        delete metaData;
    }

    delete this->agentIDFactory;
}

const map<string, vector<pair<int, int>>> *const RobotExecutableRegistry::getBundlesMap()
{
    if (bundlesMap.size() == 0)
    {
        // Read bundles from Processes.conf
        auto bundlesSections = (*this->sc)["ProcessManaging"]->getSections("Processes.Bundles", NULL);
        for (auto bundleName : (*bundlesSections))
        {
            vector<string> processList = (*this->sc)["ProcessManaging"]->getList<string>(
                "Processes.Bundles", bundleName.c_str(), "processList", NULL);
            vector<string> processParamsList = (*this->sc)["ProcessManaging"]->getList<string>(
                "Processes.Bundles", bundleName.c_str(), "processParamsList", NULL);
            if (processList.size() != processParamsList.size())
            {
                cerr << "RobotExecutableReg: Number of processes does not match the number of parameter sets for the "
                        "bundle '"
                     << bundleName << "' in the ProcessManaging.conf!" << endl;
                continue;
            }

            for (int i = 0; i < processList.size(); i++)
            {
                this->bundlesMap[bundleName].push_back(
                    pair<int, int>(stoi(processList[i]), stoi(processParamsList[i])));
            }
            cout << "RobotExecutableReg: Bundle '" << bundleName << "' has " << this->bundlesMap[bundleName].size()
                 << " processes." << endl;
        }
    }
    return &bundlesMap;
}

bool RobotExecutableRegistry::getRobotName(const IAgentID *agentID, string &robotName)
{
    for (auto robotMetaData : this->robotList)
    {
        if (*(robotMetaData->agentID) == *agentID)
        {
            robotName = robotMetaData->name;
            return true;
        }
    }

    robotName = "";
    return false;
}

bool RobotExecutableRegistry::robotExists(const IAgentID *agentID)
{
    for (auto robotMetaData : this->robotList)
    {
        if (*(robotMetaData->agentID) == *agentID)
        {
            return true;
        }
    }
    return false;
}

bool RobotExecutableRegistry::robotExists(string robotName)
{
    for (auto robotMetaData : this->robotList)
    {
        if (robotMetaData->name == robotName)
        {
            return true;
        }
    }
    return false;
}

const IAgentID* RobotExecutableRegistry::getRobotId(string robotName)
{
    for (auto robotMetaData : this->robotList)
    {
        if (robotMetaData->name == robotName)
        {
            return robotMetaData->agentID;
        }
    }

    return nullptr;
}

const IAgentID* RobotExecutableRegistry::getRobotId(vector<uint8_t>& idVector, string& robotName)
{
	auto agentID = this->agentIDFactory->create(idVector);
    for (auto robotMetaData : this->robotList)
    {
    	if (*(robotMetaData->agentID) == *agentID)
    	{
    		delete agentID;
    		robotName = robotMetaData->name;
    		return robotMetaData->agentID;
    	}
    }

    delete agentID;
    robotName = "";
    return nullptr;
}

const IAgentID* RobotExecutableRegistry::getRobotId(const vector<uint8_t>& idVector)
{
	auto agentID = this->agentIDFactory->create(idVector);
    for (auto robotMetaData : this->robotList)
    {
    	if (*(robotMetaData->agentID) == *agentID)
    	{
    		delete agentID;
    		return robotMetaData->agentID;
    	}
    }

    delete agentID;
    return nullptr;
}

void RobotExecutableRegistry::addRobot(string robotName, const IAgentID *agentID)
{
    this->robotList.push_back(new RobotMetaData(robotName, agentID));
}

/**
 * Adds a robot with its configured id, if it exists. Otherwise it generates a new unique id.
 * This method allows testing with systems, which are not in the Globals.conf,
 * i.d., are no official robots.
 */
const IAgentID *RobotExecutableRegistry::addRobot(string agentName)
{
    const IAgentID *agentID;

    try
    {
        int tmpID = (*sc)["Globals"]->get<int>("Globals.Team", agentName.c_str(), "ID", NULL);
        std::vector<uint8_t> agentIDVector;
        for (int i = 0; i < sizeof(int); i++)
        {
            agentIDVector.push_back(*(((uint8_t *)&tmpID) + i));
        }
        agentID = this->agentIDFactory->create(agentIDVector);
    }
    catch (const std::runtime_error* e)
    {
        agentID = nullptr;
        bool idExists;

        do
        {
            idExists = false;
            // generates random ID
            agentID = this->agentIDFactory->generateID();
            for (auto entry : this->robotList)
            {
                if (*(entry->agentID) == *(agentID))
                {
                    idExists = true;
                    break;
                }
            }
        } while (idExists);
        std::cout << "PM Registry: Warning! Adding unknown agent " << agentName << " with ID " << agentID << "!"
                  << std::endl;
    }

    this->addRobot(agentName, agentID);
    return agentID;
}

const vector<RobotMetaData *> &RobotExecutableRegistry::getRobots() const
{
    return this->robotList;
}

bool RobotExecutableRegistry::getExecutableName(int execId, string &execName)
{
    for (auto execMetaData : this->executableList)
    {
        if (execMetaData->id == execId)
        {
            execName = execMetaData->name;
            return true;
        }
    }

    execName = "";
    return false;
}

bool RobotExecutableRegistry::getExecutableIdByExecName(string execName, int &execId)
{
    for (auto execMetaData : this->executableList)
    {
        if (execMetaData->execName == execName)
        {
            execId = execMetaData->id;
            return true;
        }
    }

    execId = 0;
    return false;
}

bool RobotExecutableRegistry::getExecutableId(vector<string> &splittedCmdLine, int &execId)
{
    for (auto execMetaData : this->executableList)
    {
        if (execMetaData->matchSplittedCmdLine(splittedCmdLine))
        {
            execId = execMetaData->id;
            return true;
        }
    }
    execId = 0;
    return false;
}

bool RobotExecutableRegistry::executableExists(int execId)
{
    for (auto execMetaData : this->executableList)
    {
        if (execMetaData->id == execId)
        {
            return true;
        }
    }

    return false;
}

bool RobotExecutableRegistry::executableExists(string execName)
{
    for (auto execMetaData : this->executableList)
    {
        if (execMetaData->name == execName)
        {
            return true;
        }
    }

    return false;
}

/**
 * This method registers the given executable, if it is listed in the ProcessManaging.conf file.
 * @param execName
 * @return -1, if the executable is not registered, due to some error. Otherwise, it returns the registered id.
 */
int RobotExecutableRegistry::addExecutable(string execSectionName)
{
    if (this->executableExists(execSectionName))
    {
        cerr << "RobotExecutableRegistry: The executable '" << execSectionName << "' is already registered!" << endl;
        return -1;
    }

    SystemConfig *sc = SystemConfig::getInstance();
    int execId;
    string processMode;
    string execName;
    string absExecName;
    string rosPackage = "NOT-FOUND"; // optional
    string prefixCmd = "NOT-FOUND";  // optional

    try
    {
        execId =
            (*sc)["ProcessManaging"]->get<int>("Processes.ProcessDescriptions", execSectionName.c_str(), "id", NULL);
        processMode = (*sc)["ProcessManaging"]->get<string>("Processes.ProcessDescriptions", execSectionName.c_str(),
                                                            "mode", NULL);
        execName = (*sc)["ProcessManaging"]->get<string>("Processes.ProcessDescriptions", execSectionName.c_str(),
                                                         "execName", NULL);
        rosPackage = (*sc)["ProcessManaging"]->tryGet<string>("NOT-FOUND", "Processes.ProcessDescriptions",
                                                              execSectionName.c_str(), "rosPackage", NULL);
        prefixCmd = (*sc)["ProcessManaging"]->tryGet<string>("NOT-FOUND", "Processes.ProcessDescriptions",
                                                             execSectionName.c_str(), "prefixCmd", NULL);
    }
    catch (runtime_error &e)
    {
        cerr << "PM-Registry: Cannot add executable '" << execSectionName
             << "', because of faulty values in ProcessManaging.conf!" << endl;
        return -1;
    }

    // create absolute executable name, if possible
    if (rosPackage.compare("NOT-FOUND") != 0 && prefixCmd.compare("roslaunch") != 0)
    {
        string cmd = "catkin_find --first-only --libexec " + rosPackage;
        absExecName = supplementary::ConsoleCommandHelper::exec(cmd.c_str());

        if (absExecName.length() > 1)
        {
            absExecName = absExecName.substr(0, absExecName.length() - 1);
            absExecName = absExecName + "/" + execName;
        }
    }

    ExecutableMetaData *execMetaData =
        new ExecutableMetaData(execSectionName, execId, processMode, execName, rosPackage, prefixCmd, absExecName);
    auto paramSets = (*sc)["ProcessManaging"]->tryGetNames("NONE", "Processes.ProcessDescriptions",
                                                           execSectionName.c_str(), "paramSets", NULL);
    if (paramSets->size() > 1 || paramSets->at(0) != "NONE")
    {
        for (string paramSetKeyString : (*paramSets))
        {
            try
            {
                int paramSetKey = stoi(paramSetKeyString);
                auto paramSetValues =
                    (*sc)["ProcessManaging"]->getList<string>("Processes.ProcessDescriptions", execSectionName.c_str(),
                                                              "paramSets", paramSetKeyString.c_str(), NULL);

                // first param is always the executable name
                vector<char *> currentParams;
                if (absExecName.length() > 1)
                {
                    currentParams.push_back(strdup(absExecName.c_str()));
                }
                else
                {
                    currentParams.push_back(strdup(execName.c_str()));
                }
                // transform the system config params to vector of char*, for c-compatibility.
                cout << currentParams[0] << endl;
                for (string param : paramSetValues)
                {
                    char *tmp = new char[param.size() + 1];
                    strcpy(tmp, param.c_str());
                    tmp[param.size()] = '\0';
                    currentParams.push_back(tmp);
                }
                currentParams.push_back(nullptr);

                execMetaData->addParameterSet(paramSetKey, currentParams);
            }
            catch (exception &e)
            {
                cerr << "RobotExecutableRegistry: Unable to parse parameter set \"" << paramSetKeyString
                     << "\" of process \"" << execSectionName << "\"" << endl;
                cerr << e.what() << endl;
            }
        }
    }
    else
    {
        vector<char *> currentParams;
        if (absExecName.length() > 1)
        {
            currentParams.push_back(strdup(absExecName.c_str()));
        }
        else
        {
            currentParams.push_back(strdup(execName.c_str()));
        }
        currentParams.push_back(nullptr);
        execMetaData->addParameterSet(0, currentParams);
    }

    // cout << (*execMetaData) << endl;
    this->executableList.push_back(execMetaData);
    return execId;
}

/**
 * This method is for copying meta data from an entry into a real managed executable.
 * Don't change anything in the returns object.
 * @param execName is the name of the demanded entry.
 * @return The demanded entry, if it exists. nullptr, otherwise.
 */
ExecutableMetaData const *const RobotExecutableRegistry::getExecutable(string execName) const
{
    for (auto execEntry : this->executableList)
    {
        if (execEntry->name == execName)
        {
            return execEntry;
        }
    }
    return nullptr;
}

/**
 * This method is for copying meta data from an entry into a real managed executable.
 * Don't change anything in the returns object.
 * @param execId is the id of the demanded entry.
 * @return The demanded entry, if it exists. nullptr, otherwise.
 */
ExecutableMetaData const *const RobotExecutableRegistry::getExecutable(int execId) const
{
    for (auto execEntry : this->executableList)
    {
        if (execEntry->id == execId)
        {
            return execEntry;
        }
    }
    return nullptr;
}

/**
 * For accessing the internal data structure of executable meta data entries.
 * @return The internal data structure of executable meta data entries.
 */
const vector<ExecutableMetaData *> &RobotExecutableRegistry::getExecutables() const
{
    return this->executableList;
}

void RobotExecutableRegistry::setInterpreters(vector<string> interpreter)
{
    this->interpreter = interpreter;
}

} /* namespace supplementary */
