#include "engine/allocationauthority/EntryPointRobotPair.h"

namespace alica
{

EntryPointRobotPair::EntryPointRobotPair(const EntryPoint* ep, uint64_t r)
        : _entryPoint(ep)
        , _robot(r)
{
}

const EntryPoint* EntryPointRobotPair::getEntryPoint() const
{
    return _entryPoint;
}

void EntryPointRobotPair::setEntryPoint(const EntryPoint* entryPoint)
{
    _entryPoint = entryPoint;
}

void EntryPointRobotPair::setRobot(uint64_t robot)
{
    _robot = robot;
}

bool EntryPointRobotPair::operator==(const EntryPointRobotPair& other) const
{
    if (other._entryPoint != _entryPoint) { // entrypoints are supposed to be uniquely instantiated.
        return false;
    }
    return (other._robot == _robot);
}

} /* namespace alica */
