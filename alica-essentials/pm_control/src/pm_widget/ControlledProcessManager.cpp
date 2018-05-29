#include <SystemConfig.h>
#include <process_manager/RobotExecutableRegistry.h>

#include "pm_widget/ControlledProcessManager.h"
#include "pm_widget/ControlledRobot.h"
#include "ui_RobotProcessesWidget.h"

using std::chrono::duration;
using std::cout;
using std::endl;
using std::string;

namespace pm_widget
{
ControlledProcessManager::ControlledProcessManager(string processManagerName, const supplementary::AgentID* processManagerId, QBoxLayout* parentLayout)
    : name(processManagerName)
    , id(processManagerId)
    , pmRegistry(supplementary::RobotExecutableRegistry::get())
    , parentLayout(parentLayout)
{
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    this->msgTimeOut = duration<double>((*sc)["ProcessManaging"]->get<unsigned long>("PMControl.timeLastMsgReceivedTimeOut", NULL));
}

ControlledProcessManager::~ControlledProcessManager()
{
    for (auto& controlledRobotEntry : this->controlledRobotsMap) {
        delete controlledRobotEntry.second;
    }
}

void ControlledProcessManager::handleProcessStats(std::pair<std::chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr> timePstsPair)
{
    this->timeLastMsgReceived = timePstsPair.first;
    for (auto& processStat : timePstsPair.second->processStats) {
        // get the corresponding controlled robot
        auto agentID = this->pmRegistry->getRobotId(processStat.robotId.id);
        ControlledRobot* controlledRobot = this->getControlledRobot(agentID);
        if (controlledRobot != nullptr) {
            // call the controlled robot to update its corresponding process statistics.
            controlledRobot->handleProcessStat(timePstsPair.first, processStat, agentID);
        }
    }
}

ControlledRobot* ControlledProcessManager::getControlledRobot(const supplementary::AgentID* robotId)
{
    auto controlledRobotEntry = this->controlledRobotsMap.find(robotId);
    if (controlledRobotEntry != this->controlledRobotsMap.end()) { // robot is already known
        return controlledRobotEntry->second;
    } else { // robot is unknown
        string robotName;
        if (this->pmRegistry->getRobotName(robotId, robotName)) {
            std::cout << "ControlledPM: Create new ControlledRobot " << robotName << " (ID: " << *robotId << ")" << std::endl;

            ControlledRobot* controlledRobot = new ControlledRobot(robotName, robotId, this->id);
            this->controlledRobotsMap.emplace(robotId, controlledRobot);
            this->addRobot(controlledRobot->robotProcessesQFrame);
            return controlledRobot;
        } else {
            std::cerr << "ControlledPM: Received processStat from unknown robot with sender id " << *robotId << std::endl;
            return nullptr;
        }
    }
}

void ControlledProcessManager::updateGUI(std::chrono::system_clock::time_point now)
{
    for (auto robotMapIter = this->controlledRobotsMap.begin(); robotMapIter != this->controlledRobotsMap.end();) {
        if ((now - robotMapIter->second->timeLastMsgReceived) > this->msgTimeOut) { // time is over, erase controlled robot
            std::cout << "ControlledPM: The robot " << robotMapIter->second->name << " (ID: " << robotMapIter->second->agentID << ") on process manager "
                      << this->name << " (ID: " << this->id << ") seems to be dead!" << std::endl;
            delete (robotMapIter->second);
            robotMapIter = this->controlledRobotsMap.erase(robotMapIter);
        } else { // message arrived before timeout, update its GUI
            robotMapIter->second->updateGUI(now);
            ++robotMapIter;
        }
    }
}

void ControlledProcessManager::addRobot(QFrame* robot)
{
    this->parentLayout->addWidget(robot);
}

void ControlledProcessManager::removeRobot(QFrame* robot)
{
    this->parentLayout->removeWidget(robot);
}

void ControlledProcessManager::hide()
{
    for (auto& test : this->controlledRobotsMap) {
        test.second->robotProcessesQFrame->hide();
    }
}

void ControlledProcessManager::show()
{
    for (auto& test : this->controlledRobotsMap) {
        test.second->robotProcessesQFrame->show();
    }
}

} /* namespace pm_widget */
