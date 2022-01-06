#pragma once

#include <cassert>
#include <engine/IAlicaTrace.h>
#include <optional>
#include <string>
#include <unordered_map>

namespace alicaTestTracing
{
class DummyAlicaTestTrace : public alica::IAlicaTrace
{
public:
    DummyAlicaTestTrace(const std::string& opName, std::optional<const std::string> parent) {}
    ~DummyAlicaTestTrace(){};

    void setTag(const std::string& key, const std::string& value) {}
    void setLog(std::pair<std::string, std::string> logEntry) {}
    void markError(const std::string& description) {}
    void finish() {}
    std::string context() const { return _opName; }

    std::string _opName;
};

class DummyAlicaTestTraceFactory : public alica::IAlicaTraceFactory
{
public:
    DummyAlicaTestTraceFactory(){};
    ~DummyAlicaTestTraceFactory() = default;

    std::unique_ptr<alica::IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent = std::nullopt) const
    {
        std::unique_ptr<DummyAlicaTestTrace> trace = std::make_unique<DummyAlicaTestTrace>(opName, parent);
        return trace;
    }
};
} // namespace alicaTestTracing
