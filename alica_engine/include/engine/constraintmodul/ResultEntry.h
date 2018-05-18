#pragma once

#include "engine/AlicaClock.h"
#include "engine/Types.h"
#include "engine/collections/Variant.h"
#include <mutex>
#include <unordered_map>
#include <vector>

namespace alica
{
class AlicaEngine;
struct SolverVar;
class Variable;

class ResultEntry
{
  public:
    ResultEntry() = default;
    ResultEntry(const supplementary::AgentID* robotId);

    const supplementary::AgentID* getId() const { return _id; }

    ResultEntry(const ResultEntry&) = delete;
    ResultEntry& operator=(const ResultEntry&) = delete;

    ResultEntry(ResultEntry&&);
    ResultEntry& operator=(ResultEntry&&);

    void addValue(int64_t vid, Variant result, AlicaTime time);
    void clear();
    void getCommunicatableResults(AlicaTime earliest, std::vector<SolverVar>& o_result) const;
    Variant getValue(int64_t vid, AlicaTime earliest) const;
    template <typename VarType>
    bool getValues(const std::vector<VarType*>& query, AlicaTime earliest, std::vector<Variant>& o_values) const;

  private:
    class VarValue
    {
      public:
        Variant _val;
        AlicaTime _lastUpdate;

        VarValue(Variant v, AlicaTime now)
            : _val(v)
            , _lastUpdate(now)
        {
        }
    };
    std::unordered_map<int64_t, VarValue> _values;
    mutable std::mutex _valueLock;
    AgentIDConstPtr _id;
};

template <typename VarType>
bool ResultEntry::getValues(const std::vector<VarType*>& query, AlicaTime earliest, std::vector<Variant>& o_values) const
{
    o_values.resize(query.size());
    int i = 0;
    int nans = 0;
    for (const VarType* v : query) {
        o_values[i] = getValue(v->getId(), earliest);
        if (!o_values[i].isSet()) {
            ++nans;
        }
        ++i;
    }
    return nans != i;
}

} /* namespace alica */
