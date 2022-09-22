#pragma once

#include <engine/IAlicaTrace.h>

#include <jaegertracing/Tracer.h>
#include <memory>
#include <string>
#include <unordered_map>

namespace alicaTracing
{
// Value can be numeric types, strings, or bools.
using RawTraceValue = opentracing::Value;

class TraceFactory : public alica::IAlicaTraceFactory
{
public:
    TraceFactory(const std::string& serviceName, const std::string& configFilePath,
            const std::unordered_map<std::string, RawTraceValue>& defaultTags = std::unordered_map<std::string, RawTraceValue>());
    TraceFactory(const TraceFactory&) = delete;
    TraceFactory(TraceFactory&&) = delete;
    TraceFactory& operator=(const TraceFactory&) = delete;
    TraceFactory&& operator=(TraceFactory&&) = delete;

    ~TraceFactory();
    std::unique_ptr<alica::IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent = std::nullopt) const override;
    void setGlobalContext(const std::string& globalContext) override;
    void unsetGlobalContext() override;
private:
    bool _initialized = false;
    std::unordered_map<std::string, RawTraceValue> _defaultTags;
    std::string _serviceName;

    mutable std::mutex _mutex;
    std::optional<std::string> _globalContext;
};

} // namespace alicaTracing
