#include "tracing/TraceFactory.h"
#include "engine/logging/Logging.h"
#include "tracing/Trace.h"

#include <exception>

#include "opentelemetry/exporters/otlp/otlp_grpc_exporter_factory.h"
#include "opentelemetry/sdk/trace/simple_processor_factory.h"
#include "opentelemetry/sdk/trace/tracer_provider_factory.h"
#include "opentelemetry/trace/provider.h"
#include "opentelemetry/trace/span_startoptions.h"

#include "opentelemetry/sdk/trace/tracer_provider.h"

namespace otlp = opentelemetry::exporter::otlp;
namespace nostd = opentelemetry::nostd;
namespace sdktrace = opentelemetry::sdk::trace;
namespace trace = opentelemetry::trace;

namespace alicaTracing
{

TraceFactory::TraceFactory(const std::string& serviceName, const std::string& configFilePath, const std::unordered_map<std::string, OTLTraceValue>& defaultTags)
        : _defaultTags(defaultTags)
{
    alica::Logging::logInfo(LOGNAME) << __func__ << " Initializing tracing for service " << serviceName;
    _serviceName = serviceName;
    try {
        auto configYAML = YAML::LoadFile(configFilePath);

        otlp::OtlpGrpcExporterOptions options;
        options.endpoint = configYAML["reporter.server_addr"].as<std::string>() + ":" + configYAML["reporter.server_port"].as<std::string>();
        options.use_ssl_credentials = configYAML["reporter.use_ssl_credentials"].as<bool>();
        options.ssl_credentials_cacert_as_string = "ssl-certificate";

        auto exporter = otlp::OtlpGrpcExporterFactory::Create(options);
        auto processor = sdktrace::SimpleSpanProcessorFactory::Create(std::move(exporter));

        _provider = sdktrace::TracerProviderFactory::Create(std::move(processor));
        _tracer = _provider->GetTracer(_serviceName);
    } catch (std::exception& e) {
        alica::Logging::logInfo(LOGNAME) << __func__ << " Failed to initialize OTLP " << e.what();
        throw e;
    }
    _initialized = true;
    alica::Logging::logInfo(LOGNAME) << __func__ << " tracing for service " << _serviceName;
}

TraceFactory::~TraceFactory()
{
    alica::Logging::logInfo(LOGNAME) << __func__ << " Terminating tracing for service " << _serviceName;
    // allow termination to propogate,
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    _provider = trace::Provider::GetTracerProvider();
    if (_provider) {
        static_cast<sdktrace::TracerProvider*>(_provider.get())->ForceFlush();
    }

    std::shared_ptr<opentelemetry::trace::TracerProvider> none;
    trace::Provider::SetTracerProvider(none);
}

std::unique_ptr<alica::IAlicaTrace> TraceFactory::create(const std::string& opName, std::optional<const std::string> parent) const
{
    assert(!opName.empty());

    if (!_initialized) {
        return nullptr;
    }

    std::optional<std::string> applicableParent = parent;
    if (!applicableParent) {
        std::lock_guard<std::mutex> lck(_mutex);
        applicableParent = _globalContext; // Note: _globalContext may be intentionally empty
    }

    std::unique_ptr<Trace> trace = std::unique_ptr<Trace>(new Trace(createSpan(opName, applicableParent)));
    for (const auto& defaultTag : _defaultTags) {
        trace->setTag(defaultTag.first, defaultTag.second);
    }

    return trace;
}

OTLSpanPtr TraceFactory::createSpan(const std::string& opName, std::optional<const alica::TraceContext> parent) const
{
    assert(!opName.empty());

    if (!_initialized) {
        return OTLSpanPtr();
    }

    OTLSpanPtr span;
    if (parent) {
        trace::SpanContext context(trace::TraceId(parent->trace_id), trace::SpanId(parent->span_id), trace::TraceFlags(parent->trace_flags), true,
                opentelemetry::trace::TraceState::FromHeader(parent->trace_state));
        trace::StartSpanOptions options;
        options.parent = context;
        span = _tracer->StartSpan(opName, options);
    } else {
        span = _tracer->StartSpan(opName);
    }

    for (const auto& defaultTag : _defaultTags) {
        span->SetAttribute(defaultTag.first, defaultTag.second);
    }
    return span;
}

void TraceFactory::setGlobalContext(const std::string& globalContext)
{
    std::lock_guard<std::mutex> lck(_mutex);
    _globalContext = globalContext;
}

void TraceFactory::unsetGlobalContext()
{
    std::lock_guard<std::mutex> lck(_mutex);
    _globalContext.reset();
}
} // namespace alicaTracing
