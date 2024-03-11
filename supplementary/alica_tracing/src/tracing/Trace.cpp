#include "tracing/Trace.h"

#include <algorithm>
#include <iterator>
#include <string_view>
#include <utility>
#include <variant>

#include <opentracing/string_view.h>
#include <opentracing/util.h>

namespace alicaTracing
{
namespace
{
auto prepareStringView(std::string_view value)
{
    return opentracing::string_view(value.data(), value.length());
}

struct TraceValueConverter
{
    template <typename T>
    auto operator()(T&& value) const
    {
        return RawTraceValue(std::forward<T>(value));
    }
    auto operator()(std::string_view value) const
    {
        // The rest of the tracing process didn't work properly with string views, so construct the string here.
        return RawTraceValue(std::string(value));
    }
};

template <typename T>
RawTraceValue prepareRawTraceValue(T&& value)
{
    return std::visit(TraceValueConverter{}, std::forward<T>(value));
}
} // namespace

Trace::Trace(const std::string& opName, std::optional<const std::string> parent)
{
    if (parent) {
        std::stringstream ss(parent.value());
        jaegertracing::SpanContext parentCtx = jaegertracing::SpanContext::fromStream(ss);
        _rawTrace = opentracing::Tracer::Global()->StartSpan(opName, {opentracing::ChildOf(&parentCtx)});
    } else {
        _rawTrace = opentracing::Tracer::Global()->StartSpan(opName);
    }
}

Trace::~Trace()
{
    _rawTrace->Finish();
}

void Trace::setTag(std::string_view key, TraceValue value)
{
    _rawTrace->SetTag(prepareStringView(key), prepareRawTraceValue(std::move(value).variant));
}

void Trace::setTag(const std::string& key, const RawTraceValue& value)
{
    _rawTrace->SetTag(key, value);
}

void Trace::log(const std::unordered_map<std::string_view, TraceValue>& fields)
{
    using RawFields = std::vector<std::pair<opentracing::string_view, RawTraceValue>>;
    RawFields raw_fields;
    raw_fields.reserve(fields.size());
    std::transform(begin(fields), end(fields), std::back_inserter(raw_fields),
            [](const auto& v) { return std::make_pair(prepareStringView(v.first), prepareRawTraceValue(v.second.variant)); });
    _rawTrace->Log(opentracing::SystemClock::now(), raw_fields);
}

void Trace::markError(std::string_view description)
{
    setTag("error", TraceValue(true));
    setTag("error.description", TraceValue(description));
}

std::string Trace::context() const
{
    const auto& jctx = dynamic_cast<const jaegertracing::SpanContext&>(_rawTrace->context());
    std::stringstream ss;
    jctx.print(ss);
    return ss.str();
}

void Trace::finish()
{
    setTag("endTime", RawTraceValue(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
    _rawTrace->Finish();
}

} // namespace alicaTracing
