#include "tracing/TraceFactory.h"
#include "tracing/Trace.h"

#include <exception>
#include <jaegertracing/Tracer.h>
#include <ros/ros.h>

namespace alicaTracing
{

TraceFactory::TraceFactory(const std::string& serviceName, const std::string& configFilePath, const std::unordered_map<std::string, RawTraceValue>& defaultTags)
    : _defaultTags(defaultTags)
{
    ROS_INFO_STREAM_NAMED(__func__, "Initializing tracing for service " << serviceName);
    _serviceName = serviceName;
    try {
        auto configYAML = YAML::LoadFile(configFilePath);
        auto config = jaegertracing::Config::parse(configYAML);
        auto tracer = jaegertracing::Tracer::make(_serviceName, config, jaegertracing::logging::consoleLogger());
        opentracing::Tracer::InitGlobal(std::static_pointer_cast<opentracing::Tracer>(tracer));
    } catch (std::exception& e) {
        ROS_ERROR_STREAM_NAMED(__func__, "Failed to initialize jaeger: " << e.what());
        throw e;
    }
    _initialized = true;
    ROS_INFO_STREAM_NAMED(__func__, "Initialized tracing for service " << _serviceName);
}

TraceFactory::~TraceFactory()
{
    ROS_INFO_STREAM_NAMED(__func__, "Terminating tracing for service " << _serviceName);
    // allow termination to propogate,
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    opentracing::Tracer::Global()->Close();
};

std::unique_ptr<alica::IAlicaTrace> TraceFactory::create(const std::string& opName, std::optional<const std::string> parent) const
{
    assert(!opName.empty());

    if (!_initialized) {
        return std::make_unique<Trace>();
    }

    std::unique_ptr<Trace> trace = std::make_unique<Trace>(opName, parent);
    for (const auto& defaultTag : _defaultTags) {
        trace->setTag(defaultTag.first, defaultTag.second);
    }

    // used for immediately reporting traces instead of waiting for termination of the master plan
    // TODO: is this required? & if so find a better way of knowing if the plan is the master plan
    if (opName == "Master") {
        trace->finish();
    }

    return trace;
}

} // namespace alicaTracing
