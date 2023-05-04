#include "tracing/Trace.h"

#include <algorithm>
#include <iterator>
#include <string_view>
#include <utility>
#include <variant>

#include "opentelemetry/sdk/trace/simple_processor_factory.h"
#include "opentelemetry/sdk/trace/tracer_provider.h"
#include "opentelemetry/sdk/trace/tracer_provider_factory.h"
#include "opentelemetry/trace/provider.h"
#include "opentelemetry/trace/span_startoptions.h"

#include "SpanWrapper.hpp"
#include "TraceUtils.hpp"

namespace nostd = opentelemetry::nostd;
namespace sdktrace = opentelemetry::sdk::trace;
namespace trace = opentelemetry::trace;

// Value can be numeric types, strings, or bools.
using OTELTraceValue = opentelemetry::v1::common::AttributeValue;
using OTELSpan = opentelemetry::trace::Span;
using OTELSpanPtr = opentelemetry::nostd::shared_ptr<OTELSpan>;

namespace alicaTracing
{

Trace::Trace(SpanWrapper&& span)
        : _span(std::make_unique<SpanWrapper>(span))
{
}

Trace::~Trace()
{
    _span->_span->End();
}

void Trace::setTag(std::string_view key, TraceValue value)
{
    _span->_span->SetAttribute(prepareStringView(key), prepareOTELTraceValue(extractVariant(std::move(value))));
}

// void Trace::setTag(const std::string& key, const OTELTraceValue& value)
// {
//     _span->_span->SetAttribute(key, value);
// }

void Trace::log(const std::unordered_map<std::string_view, TraceValue>& fields, const std::string& event_name)
{
    using RawFields = std::vector<std::pair<nostd::string_view, OTELTraceValue>>;
    RawFields raw_fields;
    raw_fields.reserve(fields.size());
    std::transform(begin(fields), end(fields), std::back_inserter(raw_fields),
            [](const auto& v) { return std::make_pair(prepareStringView(v.first), prepareOTELTraceValue(extractVariant(v.second))); });
    _span->_span->AddEvent(event_name, raw_fields);
}

void Trace::markError(std::string_view description)
{
    setTag("error", TraceValue(true));
    setTag("error.description", TraceValue(description));
}

alica::TraceContext Trace::context() const
{
    if (!_span) {
        return std::string();
    }

    alica::TraceContext amrCtx;
    amrCtx.trace_id.resize(16);
    _span->_span->GetContext().trace_id().CopyBytesTo(nostd::span<uint8_t, 16UL>(amrCtx.trace_id.data(), amrCtx.trace_id.size()));
    amrCtx.span_id.resize(8);
    _span->_span->GetContext().span_id().CopyBytesTo(nostd::span<uint8_t, 8UL>(amrCtx.span_id.data(), amrCtx.span_id.size()));
    std::vector<uint8_t> trace_flags_vec;
    trace_flags_vec.resize(1);
    _span->_span->GetContext().trace_flags().CopyBytesTo(nostd::span<uint8_t, 1UL>(trace_flags_vec.data(), trace_flags_vec.size()));
    amrCtx.trace_flags = trace_flags_vec[0];
    amrCtx.trace_state = _span->_span->GetContext().trace_state()->ToHeader();
    return amrCtx;
}

void Trace::finish()
{
    _span->_span->SetAttribute(prepareStringView("endTime"),
            OTELTraceValue(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
    _span->_span->End();
}

} // namespace alicaTracing
