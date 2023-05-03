#include "tracing/TraceFactory.h"
#include "tracing/Trace.h"

#include "engine/logging/Logging.h"

#include <exception>

#include "opentelemetry/exporters/jaeger/jaeger_exporter_factory.h"
#include "opentelemetry/sdk/trace/batch_span_processor_factory.h"
#include "opentelemetry/sdk/trace/tracer_provider.h"
#include "opentelemetry/sdk/trace/tracer_provider_factory.h"
#include "opentelemetry/trace/provider.h"
#include "opentelemetry/trace/span_startoptions.h"

#include "SpanWrapper.hpp"
#include "TraceUtils.hpp"

namespace jaeger = opentelemetry::exporter::jaeger;
namespace nostd = opentelemetry::nostd;
namespace sdktrace = opentelemetry::sdk::trace;
namespace trace = opentelemetry::trace;

// Value can be numeric types, strings, or bools.
using OTELTraceValue = opentelemetry::v1::common::AttributeValue;

using OTELSpan = opentelemetry::trace::Span;
using OTELSpanPtr = opentelemetry::nostd::shared_ptr<OTELSpan>;
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

        jaeger::JaegerExporterOptions options;
        options.endpoint = configYAML["reporter"]["server_addr"].as<std::string>();
        options.server_port = configYAML["reporter"]["server_port"].as<uint16_t>();
        auto exporter = jaeger::JaegerExporterFactory::Create(options);

        auto processor_opts = sdk::trace::BatchSpanProcessorOptions();
        processor_opts.max_export_batch_size = 5;
        processor_opts.max_queue_size = 5;
        processor_opts.schedule_delay_millis = std::chrono::milliseconds(256);
        auto processor = sdktrace::BatchSpanProcessorFactory::Create(std::move(exporter), processor_opts);

        _impl->_provider = sdktrace::TracerProviderFactory::Create(std::move(processor));
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
    _impl->_provider = trace::Provider::GetTracerProvider();
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

    std::unique_ptr<Trace> trace = std::unique_ptr<Trace>(new Trace(createSpan(opName, applicableParent)));
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
            trace::SpanContext context(trace::TraceId(parent->trace_id), trace::SpanId(parent->span_id), trace::TraceFlags(parent->trace_flags), true,
                    opentelemetry::trace::TraceState::FromHeader(parent->trace_state));
            trace::StartSpanOptions options;
            options.parent = context;
            span = nostd::shared_ptr<OTELSpan>(_impl->_provider->GetTracer(_impl->_serviceName)->StartSpan(opName, options).get());
        } else {
            span = nostd::shared_ptr<OTELSpan>(_impl->_provider->GetTracer(_impl->_serviceName)->StartSpan(opName).get());
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
