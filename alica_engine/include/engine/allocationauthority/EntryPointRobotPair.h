#pragma once

#include <memory>

#include "supplementary/AgentID.h"
#include "engine/model/EntryPoint.h"

namespace alica {

/**
 * A simple helper class for conflict detection
 */
class EntryPointRobotPair {
public:
    EntryPointRobotPair(EntryPoint* ep, const supplementary::AgentID* r);
    virtual ~EntryPointRobotPair();
    EntryPoint* getEntryPoint() const;
    void setEntryPoint(EntryPoint* entryPoint);
    const supplementary::AgentID* getRobot() const;
    void setRobot(const supplementary::AgentID* robot);
    bool operator==(const EntryPointRobotPair& o) const;
    bool operator!=(const EntryPointRobotPair& o) const {
        return !(*this == o);
    }

protected:
    EntryPoint* _entryPoint;  // TODO: should be const
    const supplementary::AgentID* _robot;
};

} /* namespace alica */

namespace std {
template <>
struct hash<alica::EntryPointRobotPair> {
    typedef alica::EntryPointRobotPair argument_type;
    typedef std::size_t value_type;

    value_type operator()(argument_type& eprp) const {
        return std::hash<long int>()(eprp.getEntryPoint()->getId()) + supplementary::AgentIDHash()(eprp.getRobot());
    }
};
}  // namespace std
