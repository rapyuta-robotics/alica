#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace alica
{

class IAlicaTrace
{
    // Integral values need to be disambiguated to convert to their largest variants.
    template <typename T>
    using PromotedType = std::conditional_t<std::is_integral_v<std::decay_t<T>> && !std::is_same_v<std::decay_t<T>, bool>,
            std::conditional_t<std::is_signed_v<std::decay_t<T>>, long long int, unsigned long long int>,
            std::conditional_t<std::is_constructible_v<std::string_view, T>, std::string_view, T>>;

public:
    class TraceValue
    {
        using Variant = std::variant<bool, long long int, unsigned long long int, double, std::string_view>;

    public:
        template <typename T, typename = std::enable_if_t<std::is_constructible_v<Variant, PromotedType<T&&>>>>
        TraceValue(T&& val)
                : variant(static_cast<PromotedType<decltype(val)>>(std::forward<T>(val)))
        {
        }

        Variant variant;
    };

public:
    virtual ~IAlicaTrace() = default;
    virtual void setTag(std::string_view key, TraceValue value) = 0;
    void setLog(const std::pair<std::string_view, TraceValue>& fields) { log({{fields.first, fields.second}}); }
    virtual void log(const std::unordered_map<std::string_view, TraceValue>& fields) = 0;
    virtual void markError(std::string_view description) = 0;
    // Explicitly set the trace as finished. Any calls to setTag, setLog & markError after this call should
    // leave the trace in a valid but unspecified state. Calling context on a finished trace is a valid operation
    virtual void finish() = 0;
    virtual std::string context() const = 0;
};

namespace tracing
{
struct SpanLink
{
    std::string context;
    // Warning:  Ensure string_views in attributes are valid for the lifetime of the span link
    std::unordered_map<std::string, IAlicaTrace::TraceValue> attributes;
};

struct SpanStartOptions
{
    std::optional<std::string> parentContext;
    std::vector<SpanLink> links;
};
} // namespace tracing

class IAlicaTraceFactory
{
public:
    // Two argument version of create.  Deprecated.
    virtual std::unique_ptr<IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent = std::nullopt) const { return nullptr; }
    virtual std::unique_ptr<IAlicaTrace> create(const std::string& opName, const tracing::SpanStartOptions& options) const
    {
        // Concrete implementation provided for backwards compatibility
        return create(opName, options.parentContext);
    }
    virtual void setGlobalContext(const std::string& globalContext) = 0;
    virtual void unsetGlobalContext() = 0;
    virtual ~IAlicaTraceFactory() = default;
};

} // namespace alica
