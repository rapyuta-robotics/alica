#pragma once

#include <memory>

#include "engine/AgentIDConstPtr.h"
#include "engine/model/EntryPoint.h"

namespace alica
{

/**
 * A simple helper class for conflict detection
 */
class EntryPointRobotPair final
{
public:
    EntryPointRobotPair(const EntryPoint* ep, AgentIDConstPtr r);
    const EntryPoint* getEntryPoint() const;
    void setEntryPoint(const EntryPoint* entryPoint);
    AgentIDConstPtr getRobot() const { return _robot; }
    void setRobot(AgentIDConstPtr robot);
    bool operator==(const EntryPointRobotPair& o) const;
    bool operator!=(const EntryPointRobotPair& o) const { return !(*this == o); }

private:
    const EntryPoint* _entryPoint;
    AgentIDConstPtr _robot;
};

} /* namespace alica */

namespace std
{
template <>
struct hash<alica::EntryPointRobotPair>
{
    typedef alica::EntryPointRobotPair argument_type;
    typedef std::size_t value_type;

    value_type operator()(argument_type& eprp) const { return std::hash<long int>()(eprp.getEntryPoint()->getId()) + eprp.getRobot()->hash(); }
};
} // namespace std
