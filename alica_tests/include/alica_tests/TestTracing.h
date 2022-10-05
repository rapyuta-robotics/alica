#pragma once

#include <cassert>
#include <engine/IAlicaTrace.h>
#include <optional>
#include <string>
#include <unordered_map>

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
    ~AlicaTestTrace(){};

    void setTag(const std::string& key, const std::string& value) { _wm->tracingTags.push_back({key, value}); }
    void setLog(std::pair<std::string, std::string> logEntry)
    {
        assert(_wm);
        _wm->tracingLogs.push_back({logEntry.first, logEntry.second});
    }
    void markError(const std::string& description)
    {
        assert(_wm);

        _wm->tracingTags.push_back({"error", "true"});
        _wm->tracingTags.push_back({"error.description", description});
    }
    void finish() {}

    void setWorldModel(alica_test::SchedWM* wm) { _wm = wm; };
    std::string context() const { return _opName; }

    std::string _opName;
    std::string _parent;
    alica_test::SchedWM* _wm;
};

class AlicaTestTraceFactory : public alica::IAlicaTraceFactory
{
public:
    AlicaTestTraceFactory(){};
    ~AlicaTestTraceFactory() = default;
    void setGlobalContext(const std::string& globalContext) override {}
    void unsetGlobalContext() override {}
    void setWorldModel(alica::IAlicaWorldModel* wm) { _wm = dynamic_cast<alica_test::SchedWM*>(wm); };
    alica_test::SchedWM* _wm;

    std::unique_ptr<alica::IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent = std::nullopt) const
    {
        assert(_wm);

        assert(!opName.empty());
        std::unique_ptr<AlicaTestTrace> trace = std::make_unique<AlicaTestTrace>(opName, parent);
        trace->setWorldModel(_wm);
        if (parent.has_value()) {

            _wm->tracingParents[opName] = *parent;
        }
        return trace;
    }
};
} // namespace alicaTestTracing
