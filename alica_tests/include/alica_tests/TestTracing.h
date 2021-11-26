#pragma once

#include <engine/IAlicaTrace.h>
#include <string>
#include <unordered_map>
#include <optional>
#include <cassert>

namespace alicaTestTracing
{
class AlicaTestTrace : public alica::IAlicaTrace
{
public:
    AlicaTestTrace(const std::string& opName, std::optional<const std::string> parent)
        : _opName(opName)
    {
        if (parent) {
            _parent = *parent;
        }
    }
    ~AlicaTestTrace() {};

    void setTag(const std::string& key, const std::string& value)
    {
        alicaTests::TestWorldModel::getOne()->tracingTags.push_back({key, value});
    }
    void setLog(std::pair<std::string, std::string> logEntry)
    {
        alicaTests::TestWorldModel::getOne()->tracingLogs.push_back({logEntry.first, logEntry.second});
    }
    void markError(const std::string& description)
    {
        alicaTests::TestWorldModel::getOne()->tracingTags.push_back({"error", "true"});
        alicaTests::TestWorldModel::getOne()->tracingTags.push_back({"error.description", description});
    }
    std::string context() const
    {
        return _opName;
    }

    std::string _opName;
    std::string _parent;
};

class AlicaTestTraceFactory : public alica::IAlicaTraceFactory
{
public:
    AlicaTestTraceFactory(){};
    ~AlicaTestTraceFactory() = default;

    std::unique_ptr<alica::IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent = std::nullopt) const
    {
        assert(!opName.empty());
        std::unique_ptr<AlicaTestTrace> trace = std::make_unique<AlicaTestTrace>(opName, parent);
        if (parent.has_value()) {
            alicaTests::TestWorldModel::getOne()->tracingParents[opName] = *parent;
        }
        return trace;
    }
};
} // namespace alicaDummyTracing
