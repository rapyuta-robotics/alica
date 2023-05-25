#include "tracing/TraceFactory.h"
#include "tracing/Trace.h"

#include "engine/logging/Logging.h"

#include <exception>

#include "opentelemetry/exporters/otlp/otlp_grpc_exporter_factory.h"
#include "opentelemetry/sdk/trace/simple_processor_factory.h"
#include "opentelemetry/sdk/trace/tracer_provider.h"
#include "opentelemetry/sdk/trace/tracer_provider_factory.h"
#include "opentelemetry/trace/provider.h"
#include "opentelemetry/trace/span_startoptions.h"

#include "SpanWrapper.hpp"
#include "TraceUtils.hpp"

namespace otlp = opentelemetry::exporter::otlp;
namespace nostd = opentelemetry::nostd;
namespace sdktrace = opentelemetry::sdk::trace;
namespace trace = opentelemetry::trace;
namespace resource = opentelemetry::sdk::resource;

// Value can be numeric types, strings, or bools.
using OTELTraceValue = opentelemetry::v1::common::AttributeValue;

using OTELSpan = opentelemetry::trace::Span;
using OTELSpanPtr = opentelemetry::nostd::shared_ptr<OTELSpan>;
using OTELTracerPtr = opentelemetry::nostd::shared_ptr<opentelemetry::v1::trace::Tracer>;
using OTELTracerProviderPtr = opentelemetry::nostd::shared_ptr<opentelemetry::v1::trace::TracerProvider>;

namespace alicaTracing
{

struct TraceFactory::TraceFactoryImpl
{
    bool _initialized = false;
    std::unordered_map<std::string, AlicaTraceValue> _defaultTags;
    std::string _serviceName;

    std::optional<std::string> _globalContext;

    OTELTracerProviderPtr _provider;
    OTELTracerPtr _tracer;
};

TraceFactory::TraceFactory(
        const std::string& serviceName, const std::string& configFilePath, const std::unordered_map<std::string, AlicaTraceValue>& defaultTags)
        : _impl(std::make_unique<TraceFactoryImpl>())
{
    _impl->_defaultTags = defaultTags;
    alica::Logging::logInfo(LOGNAME) << __func__ << " Initializing tracing for service " << serviceName;
    _impl->_serviceName = serviceName;
    try {
        auto configYAML = YAML::LoadFile(configFilePath);

        otlp::OtlpGrpcExporterOptions opts;
        opts.endpoint = configYAML["reporter"]["server_addr"].as<std::string>() + std::string(":") + configYAML["reporter"]["server_port"].as<std::string>();        
        auto exporter = otlp::OtlpGrpcExporterFactory::Create(opts); 

        auto processor = sdktrace::SimpleSpanProcessorFactory::Create(std::move(exporter));

        resource::ResourceAttributes attributes = {{"service.name", _impl->_serviceName}};
        auto resource = resource::Resource::Create(attributes);
        _impl->_provider = sdktrace::TracerProviderFactory::Create(std::move(processor), resource);

        _impl->_tracer = _impl->_provider->GetTracer(_impl->_serviceName);
    } catch (std::exception& e) {
        alica::Logging::logInfo(LOGNAME) << __func__ << " Failed to initialize OTLP " << e.what();
        throw e;
    }
    _impl->_initialized = true;
    alica::Logging::logInfo(LOGNAME) << __func__ << " tracing for service " << _impl->_serviceName;
}

TraceFactory::~TraceFactory()
{
    alica::Logging::logInfo(LOGNAME) << __func__ << " Terminating tracing for service " << _impl->_serviceName;
    // allow termination to propogate,
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (_impl->_provider) {
        static_cast<sdktrace::TracerProvider*>(_impl->_provider.get())->ForceFlush();
    }

    std::shared_ptr<opentelemetry::trace::TracerProvider> none;
    trace::Provider::SetTracerProvider(none);
}

std::unique_ptr<alica::IAlicaTrace> TraceFactory::create(const std::string& opName, std::optional<const std::string> parent) const
{
    assert(!opName.empty());

    if (!_impl->_initialized) {
        return nullptr;
    }

    std::optional<std::string> applicableParent = parent;
    if (!applicableParent) {
        std::lock_guard<std::mutex> lck(_mutex);
        applicableParent = _impl->_globalContext; // Note: _globalContext may be intentionally empty
    }

    // std::optional<const alica::TraceContext> parent_trace_context;
    // if (applicableParent)
    //     parent_trace_context = 

    std::unique_ptr<Trace> trace = std::unique_ptr<Trace>(
            new Trace(createSpan(opName, applicableParent ? std::make_optional<const alica::TraceContext>(alica::TraceContext(*applicableParent)) : std::nullopt)));
    for (const auto& defaultTag : _impl->_defaultTags) {
        trace->setTag(defaultTag.first, defaultTag.second);
    }

    return trace;
}

SpanWrapper TraceFactory::createSpan(const std::string& opName, std::optional<const alica::TraceContext> parent) const
{
    assert(!opName.empty());

    OTELSpanPtr span;

    if (_impl->_initialized) {
        if (parent) {
            std::vector<uint8_t> trace_id = parent->trace_id;
            trace_id.resize(16, 0);
            std::vector<uint8_t> span_id = parent->span_id;
            span_id.resize(8, 0);
            trace::SpanContext context(trace::TraceId(trace_id), trace::SpanId(span_id), trace::TraceFlags(parent->trace_flags), true,
                    opentelemetry::trace::TraceState::FromHeader(parent->trace_state));
            trace::StartSpanOptions options;
            options.parent = context;
            span = _impl->_tracer->StartSpan(opName, options);
        } else {
            span = _impl->_tracer->StartSpan(opName);
        }
    }

    return SpanWrapper(span);
}

void TraceFactory::setGlobalContext(const std::string& globalContext)
{
    std::lock_guard<std::mutex> lck(_mutex);
    _impl->_globalContext = globalContext;
}

void TraceFactory::unsetGlobalContext()
{
    std::lock_guard<std::mutex> lck(_mutex);
    _impl->_globalContext.reset();
}
} // namespace alicaTracing
