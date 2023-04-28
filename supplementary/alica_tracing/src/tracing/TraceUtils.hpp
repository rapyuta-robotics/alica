#pragma once

#include <opentelemetry/common/attribute_value.h>
#include <opentelemetry/nostd/string_view.h>
#include <string_view>

using namespace opentelemetry::v1;
using OTLTraceValue = opentelemetry::v1::common::AttributeValue;

namespace alicaTracing
{

[[maybe_unused]] static nostd::string_view prepareStringView(std::string_view value)
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
static OTLTraceValue prepareOTLTraceValue(T&& value)
{
    return std::visit(TraceValueConverter{}, std::forward<T>(value));
}

} // namespace alicaTracing
