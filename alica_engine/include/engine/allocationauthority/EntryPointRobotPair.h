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
    EntryPointRobotPair(const EntryPoint* ep, uint64_t r);
    const EntryPoint* getEntryPoint() const;
    void setEntryPoint(const EntryPoint* entryPoint);
    uint64_t getRobot() const { return _robot; }
    void setRobot(uint64_t robot);
    bool operator==(const EntryPointRobotPair& o) const;
    bool operator!=(const EntryPointRobotPair& o) const { return !(*this == o); }

private:
    const EntryPoint* _entryPoint;
    uint64_t _robot;
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
