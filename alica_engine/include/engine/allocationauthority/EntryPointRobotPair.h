#pragma once

#include "engine/model/EntryPoint.h"

#include <memory>

namespace alica
{

/**
 * A simple helper class for conflict detection
 */
class EntryPointRobotPair final
{
public:
    EntryPointRobotPair(const EntryPoint* ep, alica::AgentId r);
    const EntryPoint* getEntryPoint() const;
    void setEntryPoint(const EntryPoint* entryPoint);
    alica::AgentId getRobot() const { return _robot; }
    void setRobot(alica::AgentId robot);
    bool operator==(const EntryPointRobotPair& o) const;
    bool operator!=(const EntryPointRobotPair& o) const { return !(*this == o); }

private:
    const EntryPoint* _entryPoint;
    alica::AgentId _robot;
};

} /* namespace alica */

namespace std
{
template <>
struct hash<alica::EntryPointRobotPair>
{
    typedef alica::EntryPointRobotPair argument_type;
    typedef std::size_t value_type;

    value_type operator()(argument_type& eprp) const { return std::hash<long int>()(eprp.getEntryPoint()->getId()) + eprp.getRobot(); }
};
} // namespace std
