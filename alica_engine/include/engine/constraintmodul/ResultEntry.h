#pragma once

#include "supplementary/AgentID.h"
#include "engine/Types.h"
#include "engine/collections/Variant.h"
#include <unordered_map>
#include <vector>
#include <mutex>

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
    void getCommunicatableResults(AlicaTime earliest, std::vector<SolverVar>& o_result);
    Variant getValue(int64_t vid, AlicaTime earliest);
    bool getValues(const VariableSet& query, AlicaTime earliest, std::vector<Variant>& o_values);
private:
    class VarValue {
    public:
        Variant _val;
        AlicaTime _lastUpdate;

        VarValue(Variant v, AlicaTime now)
            : _val(v)
            , _lastUpdate(now)
        {}
    };
    std::unordered_map<int64_t, VarValue> _values;
    std::mutex _valueLock;
    const supplementary::AgentID* _id;
    
};

} /* namespace alica */
