#pragma once

#include <exception>
#include <memory>
#include <string>

#include <engine/IAlicaTrace.h>
#include <engine/util/TraceContext.h>
#include <tracing/TraceFactory.h>

namespace alicaTracing
{
using OTLSpan = opentelemetry::trace::Span;
using OTLSpanPtr = opentelemetry::nostd::shared_ptr<OTLSpan>;
// Value can be numeric types, strings, or bools.
using OTLTraceValue = opentelemetry::v1::common::AttributeValue;

class Trace : public alica::IAlicaTrace
{
public:
    Trace() = delete;
    Trace(const Trace& other) = delete;
    Trace(Trace&& other) = delete;
    Trace& operator=(const Trace& other) = delete;
    Trace&& operator=(Trace&& other) = delete;

    // This class can either be used raw, inherited or can be used to compose module level trace classes
    // Destruction of this instance will mark the end time of operation, and will be reported to tracing collectors
    virtual ~Trace();

    // Tags are the key value pair you can search in tracing ui
    // If there is a pre-existing tag set for `key`, it is overwritten.
    void setTag(std::string_view key, TraceValue value) override;

    // log is a timestamped way to record key:value logging data
    // about a trace. Here's an example:
    //
    //    trace.Log({
    //        {"progress", "checkpoint 4 cleared"},
    //        {"wait.time", "waited 10 sec for agent 2"}});
    // BE CAREFUL about what you want to log,
    // These are supposed to be micro logs to be carried over the network.
    void log(const std::unordered_map<std::string_view, TraceValue>& fields, const std::string& event_name = "Log") override;

    // When operation being traced by this instance fail, call this api
    // Error traces are highlighted in tracing ui.
    void markError(std::string_view description) override;

    // Explicitly set the trace as finished. Any calls to setTag, setLog & markError after this call leaves
    // the trace in a valid but unspecified state. Calling context on a finished trace is a valid operation
    void finish() override;

    // Get the context of this trace to propogate across process boundary
    alica::TraceContext context() const override;

private:
    friend class TraceFactory;

    Trace(OTLSpanPtr&& span);

    void setTag(const std::string& key, const OTLTraceValue& value);

    OTLSpanPtr _span;
};
} // namespace alicaTracing
