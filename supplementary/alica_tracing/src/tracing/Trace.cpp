#include "tracing/Trace.h"

namespace alicaTracing
{
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

void Trace::setTag(const std::string& key, const std::string& value)
{
    _rawTrace->SetTag(key, value);
}

void Trace::setTag(const std::string& key, const RawTraceValue& value)
{
    _rawTrace->SetTag(key, value);
}

void Trace::setLog(std::pair<std::string, std::string> logEntry)
{
    _rawTrace->Log({logEntry});
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
    _rawTrace->Finish();
}

} // namespace alicaTracing