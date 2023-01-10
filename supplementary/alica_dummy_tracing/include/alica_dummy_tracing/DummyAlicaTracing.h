#pragma once

#include <engine/IAlicaTrace.h>
#include <optional>
#include <string>

namespace alicaTestTracing
{
class DummyAlicaTestTrace : public alica::IAlicaTrace
{
public:
    DummyAlicaTestTrace(const std::string& opName, std::optional<const std::string> parent) {}
    ~DummyAlicaTestTrace(){};

    void setTag(std::string_view key, TraceValue value) {}
    void setLog(const std::pair<std::string_view, TraceValue>& fields) {}
    void log(const std::unordered_map<std::string_view, TraceValue>& fields) {};
    void markError(std::string_view description) {}
    void finish() {}
    std::string context() const { return _opName; }

    std::string _opName;
};

class DummyAlicaTestTraceFactory : public alica::IAlicaTraceFactory
{
public:
    DummyAlicaTestTraceFactory(){};
    ~DummyAlicaTestTraceFactory() = default;
    void setGlobalContext(const std::string& globalContext) override {}
    void unsetGlobalContext() override {}

    std::unique_ptr<alica::IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent = std::nullopt) const
    {
        std::unique_ptr<DummyAlicaTestTrace> trace = std::make_unique<DummyAlicaTestTrace>(opName, parent);
        return trace;
    }
};
} // namespace alicaTestTracing
