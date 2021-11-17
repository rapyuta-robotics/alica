#pragma once

#include <exception>
#include <memory>
#include <string>

#include <engine/IAlicaTrace.h>
#include <jaegertracing/Tracer.h>

namespace alicaTracing
{
using RawTrace = opentracing::Span;
using RawTracePtr = std::unique_ptr<RawTrace>;

//// Value can be numeric types, strings, or bools.
using RawTraceValue = opentracing::Value;

class TraceFactory;

class Trace : public alica::IAlicaTrace
{
    friend TraceFactory;
public:
    // Create partial trace instance with optional reference to the parent.
    // Trace without parent will be reported as root of the trace and can have further children.
    // Although library doesn't validate operation name or stop any operation to be a root,
    // users of the library are supposed to follow this specs
    // https://app.diagrams.net/?libs=general;basic;arrows#G17zQOPcT-Tr2GStr2nYzvcbgGe_SeFUEi
    // Creation of this instance will set operation start time to now()
    Trace(const std::string& opName, std::optional<const std::string> parent = std::nullopt);

    Trace() {}

    Trace(Trace&& other) = default;

    Trace& operator=(Trace&& other) = default;

    // This class can either be used raw, inherited or can be used to compose module level trace classes
    // Destruction of this instance will mark the end time of operation, and will be reported to tracing collectors
    virtual ~Trace();

    // Tags are the key value pair you can search in tracing ui
    // If there is a pre-existing tag set for `key`, it is overwritten.
    void setTag(const std::string& key, const std::string& value);

    // Tags are the key value pair you can search in tracing ui
    // If there is a pre-existing tag set for `key`, it is overwritten.
    // Tag values can be numeric types, strings, or bools.
    void setTag(const std::string& key, const RawTraceValue& value);

    // setLog is a timestamped way to record key:value logging data
    // about a trace. Here's an example:
    //
    //    trace.Log({
    //        {"progress", "checkpoint 4 cleared"},
    //        {"wait.time", "waited 10 sec for agent 2"}});
    // BE CAREFUL about what you want to log,
    // These are supposed to be micro logs to be carried over the network.
    void setLog(std::pair<std::string, std::string> logEntry);

    // When operation being traced by this instance fail, call this api
    // Error traces are highlighted in tracing ui.
    void markError(const std::string& description);

    // Get the context of this trace to propogate across process boundary
    std::string context() const;

private:
    void finish();

    RawTracePtr _rawTrace;
};
} // namespace alicaTracing