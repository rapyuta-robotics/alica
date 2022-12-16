#include "tracing/Trace.h"

#include <variant>
#include <utility>

namespace alicaTracing
{
namespace
{
template <typename T>
RawTraceValue prepareRawTraceValue(T&& value)
{
    return std::visit([](auto&& v) { return RawTraceValue(std::forward<decltype(v)>(v)); }, std::forward<T>(value));
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

void Trace::setTag(const std::string& key, const TraceValue& value)
{
    _rawTrace->SetTag(key, prepareRawTraceValue(extractVariant(value)));
}

void Trace::setTag(const std::string& key, const RawTraceValue& value)
{
    _rawTrace->SetTag(key, value);
}

void Trace::setLog(const std::pair<std::string, TraceValue>& logEntry)
{
    const auto& [key, value] = logEntry;
    _rawTrace->Log({{key, prepareRawTraceValue(extractVariant(value))}});
}

void Trace::markError(const std::string& description)
{
    _rawTrace->SetTag("error", true);
    _rawTrace->SetTag("error.description", description);
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
    setTag("endTime", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    _rawTrace->Finish();
}

} // namespace alicaTracing
