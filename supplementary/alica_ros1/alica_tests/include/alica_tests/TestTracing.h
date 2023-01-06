#pragma once

#include <cassert>
#include <engine/IAlicaTrace.h>
#include <engine/blackboard/Blackboard.h>
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

    void log(const std::unordered_map<std::string_view, TraceValue>& fields) override{};
    void setTag(std::string_view key, TraceValue value) override
    {
        auto tmp = std::get<std::string_view>(extractVariant(std::move(value)));
        _wm->tracingTags.push_back({key, tmp});
    }

    void setLog(const std::pair<std::string_view, TraceValue>& fields) override
    {
        assert(_wm);
        auto tmp = extractVariant(std::move(fields.second));
        if (!std::holds_alternative<std::string_view>(tmp))
            return;
        auto value = std::get<std::string_view>(tmp);
        _wm->tracingLogs.push_back({fields.first, value});
    }
    void markError(std::string_view description) override
    {
        assert(_wm);

        _wm->tracingTags.push_back({"error", "true"});
        _wm->tracingTags.push_back({"error.description", description});
    }
    void finish() {}

    void setWorldModel(alicaTests::TestWorldModel* wm) { _wm = wm; };
    std::string context() const { return _opName; }

    std::string _opName;
    std::string _parent;
    alicaTests::TestWorldModel* _wm;
};

class AlicaTestTraceFactory : public alica::IAlicaTraceFactory
{
public:
    AlicaTestTraceFactory(){};
    ~AlicaTestTraceFactory() = default;
    void setGlobalContext(const std::string& globalContext) override {}
    void unsetGlobalContext() override {}
    void setWorldModel(alica::Blackboard* globalBlackboard)
    {
        _wm = alica::LockedBlackboardRW(*globalBlackboard).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel").get();
    };
    alicaTests::TestWorldModel* _wm;

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
