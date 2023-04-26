#include "tracing/Trace.h"

#include <algorithm>
#include <iterator>
#include <string_view>
#include <utility>
#include <variant>

#include "opentelemetry/exporters/otlp/otlp_grpc_exporter_factory.h"
#include "opentelemetry/sdk/trace/simple_processor_factory.h"
#include "opentelemetry/sdk/trace/tracer_provider_factory.h"
#include "opentelemetry/trace/provider.h"
#include "opentelemetry/trace/span_startoptions.h"

#include "opentelemetry/sdk/trace/tracer_provider.h"

namespace otlp      = opentelemetry::exporter::otlp;
namespace nostd     = opentelemetry::nostd;
namespace sdktrace  = opentelemetry::sdk::trace;
namespace trace     = opentelemetry::trace;

namespace alicaTracing
{
namespace
{
auto prepareStringView(std::string_view value)
{
    return nostd::string_view(value.data(), value.length());
}

struct TraceValueConverter
{
    template <typename T>
    auto operator()(T&& value) const
    {
        return OTLTraceValue(std::forward<T>(value));
    }
    auto operator()(std::string_view value) const
    {
        // The rest of the tracing process didn't work properly with string views, so construct the string here.
        return OTLTraceValue(std::string(value));
    }
};

template <typename T>
OTLTraceValue prepareOTLTraceValue(T&& value)
{
    return std::visit(TraceValueConverter{}, std::forward<T>(value));
}
} // namespace

Trace::Trace(OTLSpanPtr&& span)
    : _span(span)
{
    
}

Trace::~Trace()
{
    _span->End();
}

void Trace::setTag(std::string_view key, TraceValue value)
{
    _span->SetAttribute(prepareStringView(key), prepareOTLTraceValue(extractVariant(std::move(value))));
}

void Trace::setTag(const std::string& key, const OTLTraceValue& value)
{
    _span->SetAttribute(key, value);
}

void Trace::log(const std::unordered_map<std::string_view, TraceValue>& fields, const std::string& event_name)
{
    using RawFields = std::vector<std::pair<nostd::string_view, OTLTraceValue>>;
    RawFields raw_fields;
    raw_fields.reserve(fields.size());
    std::transform(begin(fields), end(fields), std::back_inserter(raw_fields),
            [](const auto& v) { return std::make_pair(prepareStringView(v.first), prepareOTLTraceValue(extractVariant(v.second))); });
    _span->AddEvent(event_name, raw_fields);
}

void Trace::markError(std::string_view description)
{
    setTag("error", TraceValue(true));
    setTag("error.description", TraceValue(description));
}

std::string Trace::context() const
{
    if (!_span) {
        return std::string();
    }

    TraceContext amrCtx;
    _span->GetContext().trace_id().CopyBytesTo(amrCtx.trace_id);
    _span->GetContext().span_id().CopyBytesTo(amrCtx.span_id);
    std::vector<decltype(amrCtx.trace_flags)> trace_flags_vec = {amrCtx.trace_flags};
    _span->GetContext().trace_flags().CopyBytesTo(trace_flags_vec);
    amrCtx.trace_state = _span->GetContext().trace_state()->ToHeader(); 
    return amrCtx.toString();
}

void Trace::finish()
{
    setTag("endTime", OTLTraceValue(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
    _span->End();
}

} // namespace alicaTracing
