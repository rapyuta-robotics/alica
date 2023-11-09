#include <opentelemetry/nostd/shared_ptr.h>
#include <opentelemetry/trace/span.h>

using namespace opentelemetry::v1;

namespace alicaTracing
{

struct SpanWrapper
{
    SpanWrapper(const nostd::shared_ptr<opentelemetry::trace::Span>& span)
            : _span(span)
    {
    }

    nostd::shared_ptr<opentelemetry::trace::Span> _span;
};

} // namespace alicaTracing
