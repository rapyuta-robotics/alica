#pragma once

#include <type_traits>
#include <memory>
#include <optional>
#include <variant>
#include <string>
#include <utility>

namespace alica
{

class IAlicaTrace
{
    // Integral values need to be disambiguated to convert to their largest variants.
    template <typename T>
    using PromotedType = std::conditional_t<std::is_integral_v<std::decay_t<T>> && !std::is_same_v<std::decay_t<T>, bool>,
            std::conditional_t<std::is_signed_v<std::decay_t<T>>, long long int, unsigned long long int>, T>;

public:
    class TraceValue
    {
        friend IAlicaTrace;

        using Variant = std::variant<bool, long long int, unsigned long long int, double, std::string>;

    public:
        template <typename T, typename = std::enable_if_t<std::is_constructible_v<Variant, PromotedType<T>>>>
        TraceValue(T&& val)
                : variant(static_cast<PromotedType<T>>(val))
        {
        }

    private:
        Variant variant;
    };

public:
    virtual ~IAlicaTrace() = default;
    virtual void setTag(const std::string& key, const TraceValue& value) = 0;
    virtual void setLog(const std::pair<std::string, TraceValue>& fields) = 0;
    virtual void markError(const std::string& description) = 0;
    // Explicitly set the trace as finished. Any calls to setTag, setLog & markError after this call should
    // leave the trace in a valid but unspecified state. Calling context on a finished trace is a valid operation
    virtual void finish() = 0;
    virtual std::string context() const = 0;

protected:
    template <typename V>
    static decltype(auto) extractVariant(V&& trace_value)
    {
        return (std::forward<V>(trace_value).variant);
    }
};

class IAlicaTraceFactory
{
public:
    virtual std::unique_ptr<IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent = std::nullopt) const = 0;
    virtual void setGlobalContext(const std::string& globalContext) = 0;
    virtual void unsetGlobalContext() = 0;
    virtual ~IAlicaTraceFactory() = default;
};

} // namespace alica
