#include "engine/allocationauthority/EntryPointRobotPair.h"

namespace alica
{

EntryPointRobotPair::EntryPointRobotPair(EntryPoint* ep, const supplementary::AgentID* r)
    : _entryPoint(ep)
    , _robot(r)
{
}

EntryPointRobotPair::~EntryPointRobotPair()
{
}

EntryPoint* EntryPointRobotPair::getEntryPoint() const
{
    return _entryPoint;
}

void EntryPointRobotPair::setEntryPoint(EntryPoint* entryPoint)
{
    _entryPoint = entryPoint;
}

const supplementary::AgentID* EntryPointRobotPair::getRobot() const
{
    return _robot;
}



void EntryPointRobotPair::setRobot(const supplementary::AgentID* robot)
{
    _robot = robot;
}

bool EntryPointRobotPair::operator==(const EntryPointRobotPair& other) const
{
    if (other._entryPoint != _entryPoint) { //entrypoints are supposed to be uniquely instantiated.
        return false;
    }
    return (*other._robot == *_robot);
}

} /* namespace alica */
