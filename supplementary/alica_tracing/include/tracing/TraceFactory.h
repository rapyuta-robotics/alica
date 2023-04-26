#pragma once

#include <engine/IAlicaTrace.h>
#include <engine/util/TraceContext.h>

#include <memory>
#include <string>
#include <unordered_map>

#include <opentelemetry/trace/provider.h>
#include <opentelemetry/trace/tracer.h>

using OTLSpan = opentelemetry::trace::Span;
using OTLSpanPtr = opentelemetry::nostd::shared_ptr<OTLSpan>;
// Value can be numeric types, strings, or bools.
using OTLTraceValue = opentelemetry::v1::common::AttributeValue;

using OTLTracerProviderPtr = opentelemetry::v1::nostd::shared_ptr<opentelemetry::v1::trace::TracerProvider>;
using OTLTracerPtr = opentelemetry::v1::nostd::shared_ptr<opentelemetry::v1::trace::Tracer>;

namespace alicaTracing
{
class TraceFactory : public alica::IAlicaTraceFactory
{
public:
    TraceFactory(const std::string& serviceName, const std::string& configFilePath,    
            const std::unordered_map<std::string, OTLTraceValue>& defaultTags = std::unordered_map<std::string, OTLTraceValue>());
    TraceFactory(const TraceFactory&) = delete;
    TraceFactory(TraceFactory&&) = delete;
    TraceFactory& operator=(const TraceFactory&) = delete;
    TraceFactory&& operator=(TraceFactory&&) = delete;

    ~TraceFactory();
    std::unique_ptr<alica::IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent = std::nullopt) const override;
    void setGlobalContext(const std::string& globalContext) override;
    void unsetGlobalContext() override;

private:    
    OTLSpanPtr createSpan(const std::string& opName, std::optional<const alica::TraceContext> parent = std::nullopt) const;

    static constexpr const char* LOGNAME = "TraceFactory";

    bool _initialized = false;
    std::unordered_map<std::string, OTLTraceValue> _defaultTags;
    std::string _serviceName;

    mutable std::mutex _mutex;
    std::optional<std::string> _globalContext;

    OTLTracerProviderPtr _provider;
    OTLTracerPtr _tracer;
};

} // namespace alicaTracing
