#pragma once

#include "supplementary/AgentID.h"
#include "engine/Types.h"
#include "engine/collections/Variant.h"
#include <unordered_map>
#include <vector>
#include <mutex>
#include <memory>

namespace alica {
class AlicaEngine;
struct SolverVar;
class Variable;

class ResultEntry {
public:
    ResultEntry(const supplementary::AgentID* robotId);

    const supplementary::AgentID* getId() const {return _id;}
    void addValue(int64_t vid, Variant result);
    void clear();
    std::shared_ptr<std::vector<SolverVar*>> getCommunicatableResults(long ttl4Communication);
    Variant getValue(int64_t vid, AlicaTime ttl4Usage);
    std::shared_ptr<std::vector<std::shared_ptr<std::vector<uint8_t>>>> getValues(
            std::shared_ptr<VariableSet> query, long ttl4Usage);
private:
    class VarValue {
    public:
        Variant val;
        ulong lastUpdate;

        VarValue(Variant v, ulong now) 
            : _val(v)
            , _lastUpdate(now)
        {}
    };

    const supplementary::AgentID* _id;
    std::unordered_map<int64_t, VarValue> _values;
    std::mutex _valueLock;
};

} /* namespace alica */
