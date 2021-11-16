#pragma once

#include "engine/model/EntryPoint.h"
#include "engine/Types.h"

#include <boost/functional/hash.hpp>
#include <memory>

namespace alica
{

/**
 * A simple helper class for conflict detection
 */
class EntryPointRobotPair final
{
public:
    EntryPointRobotPair(const EntryPoint* ep, AgentId r);
    const EntryPoint* getEntryPoint() const;
    void setEntryPoint(const EntryPoint* entryPoint);
    AgentId getRobot() const { return _robot; }
    void setRobot(AgentId robot);
    bool operator==(const EntryPointRobotPair& o) const;
    bool operator!=(const EntryPointRobotPair& o) const { return !(*this == o); }

private:
    const EntryPoint* _entryPoint;
    AgentId _robot;
};

} /* namespace alica */

namespace std
{
template <>
struct hash<alica::EntryPointRobotPair>
{
    typedef alica::EntryPointRobotPair argument_type;
    typedef std::size_t value_type;

    value_type operator()(argument_type& eprp) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, eprp.getEntryPoint()->getId());
        boost::hash_combine(seed, eprp.getRobot());
        return seed;
    }
};
} // namespace std
