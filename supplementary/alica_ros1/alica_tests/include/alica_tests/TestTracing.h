#pragma once

#include <cassert>
#include <engine/IAlicaTrace.h>
#include <engine/blackboard/Blackboard.h>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

#include <alica_tests/TestWorldModel.h>

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

    void setTag(std::string_view key, TraceValue value) override
    {
        assert(_wm);
        _wm->tracingTags.push_back({std::string(key), formatTraceValue(value)});
    }
    void log(const std::unordered_map<std::string_view, TraceValue>& fields) override
    {
        assert(_wm);
        for (const auto& p : fields) {
            _wm->tracingLogs.push_back({std::string(p.first), formatTraceValue(p.second)});
        }
    }
    void markError(std::string_view description) override
    {
        assert(_wm);

        _wm->tracingTags.push_back({"error", "true"});
        _wm->tracingTags.push_back({"error.description", std::string(description)});
    }
    void finish() override {}

    void setWorldModel(alicaTests::TestWorldModel* wm) { _wm = wm; };
    std::string context() const override { return _opName; }

    std::string _opName;
    std::string _parent;
    alicaTests::TestWorldModel* _wm;

private:
    static std::string formatTraceValue(const TraceValue& val)
    {
        return std::visit(
                [](const auto& v) {
                    std::stringstream ss;
                    ss << v;
                    return std::move(ss).str();
                },
                extractVariant(val));
    }
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
