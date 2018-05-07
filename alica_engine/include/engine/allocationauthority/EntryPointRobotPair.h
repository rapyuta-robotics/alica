#pragma once

#include <memory>

#include "supplementary/AgentID.h"
#include "engine/model/EntryPoint.h"

namespace alica {

/**
 * A simple helper class for conflict detection
 */
class EntryPointRobotPair final {
public:
    EntryPointRobotPair(const EntryPoint* ep, const supplementary::AgentID* r);
    ~EntryPointRobotPair();
    const EntryPoint* getEntryPoint() const;
    void setEntryPoint(const EntryPoint* entryPoint);
    const supplementary::AgentID* getRobot() const;
    void setRobot(const supplementary::AgentID* robot);
    bool operator==(const EntryPointRobotPair& o) const;
    bool operator!=(const EntryPointRobotPair& o) const { return !(*this == o); }

protected:
    const EntryPoint* _entryPoint;
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
