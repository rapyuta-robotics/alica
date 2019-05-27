#pragma once

#include "engine/model/EntryPoint.h"

#include <essentials/IdentifierConstPtr.h>

#include <memory>

namespace alica
{

/**
 * A simple helper class for conflict detection
 */
class EntryPointRobotPair final
{
public:
    EntryPointRobotPair(const EntryPoint* ep, essentials::IdentifierConstPtr r);
    const EntryPoint* getEntryPoint() const;
    void setEntryPoint(const EntryPoint* entryPoint);
    essentials::IdentifierConstPtr getRobot() const { return _robot; }
    void setRobot(essentials::IdentifierConstPtr robot);
    bool operator==(const EntryPointRobotPair& o) const;
    bool operator!=(const EntryPointRobotPair& o) const { return !(*this == o); }

protected:
    const EntryPoint* _entryPoint;
    essentials::IdentifierConstPtr _robot;
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
