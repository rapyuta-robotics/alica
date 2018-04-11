#pragma once

#include "supplementary/AgentID.h"
#include "engine/AlicaClock.h"

#include <list>
#include <map>
#include <vector>
#include <mutex>
#include <memory>

namespace alica {
class AlicaEngine;
struct SolverVar;
class Variable;

class ResultEntry {
public:
    ResultEntry(const supplementary::AgentID* robotId, const AlicaEngine* ae);
    virtual ~ResultEntry();

    const supplementary::AgentID* getId();
    void addValue(long vid, std::shared_ptr<std::vector<uint8_t>> result);
    void clear();
    std::shared_ptr<std::vector<SolverVar*>> getCommunicatableResults(long ttl4Communication);
    std::shared_ptr<std::vector<uint8_t>> getValue(long vid, long ttl4Usage);
    std::shared_ptr<std::vector<std::shared_ptr<std::vector<uint8_t>>>> getValues(
            std::shared_ptr<std::vector<const Variable*>> query, long ttl4Usage);

    class VarValue {
    public:
        long id;
        std::shared_ptr<std::vector<uint8_t>> val;
        AlicaTime lastUpdate;

        VarValue(long vid, std::shared_ptr<std::vector<uint8_t>> v, AlicaTime now) {
            this->id = vid;
            this->val = v;
            this->lastUpdate = now;
        }
    };

protected:
    const supplementary::AgentID* id;
    const AlicaEngine* ae;
    std::map<long, std::shared_ptr<VarValue>> values;
    std::mutex valueLock;
};

} /* namespace alica */
