#pragma once

#include <engine/IAlicaTrace.h>
#include <engine/util/TraceContext.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

using AlicaTraceValue = alica::IAlicaTrace::TraceValue;

namespace alicaTracing
{

class SpanWrapper;

class TraceFactory : public alica::IAlicaTraceFactory
{
    class TraceFactoryImpl;

public:
    TraceFactory(const std::string& serviceName, const std::string& configFilePath,
            const std::unordered_map<std::string, AlicaTraceValue>& defaultTags = std::unordered_map<std::string, AlicaTraceValue>());
    TraceFactory(const TraceFactory&) = delete;
    TraceFactory(TraceFactory&&) = delete;
    TraceFactory& operator=(const TraceFactory&) = delete;
    TraceFactory&& operator=(TraceFactory&&) = delete;

    ~TraceFactory();
    std::unique_ptr<alica::IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent = std::nullopt) const override;
    void setGlobalContext(const std::string& globalContext) override;
    void unsetGlobalContext() override;

private:
    SpanWrapper createSpan(const std::string& opName, std::optional<const alica::TraceContext> parent = std::nullopt) const;

    std::unique_ptr<TraceFactoryImpl> _impl;

    mutable std::mutex _mutex;
    static constexpr const char* LOGNAME = "TraceFactory";
};

}  // namespace alicaTracing
