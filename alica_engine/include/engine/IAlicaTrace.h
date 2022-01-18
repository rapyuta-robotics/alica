#pragma once

#include <memory>
#include <optional>
#include <string>

namespace alica
{

class IAlicaTrace
{
public:
    virtual ~IAlicaTrace() = default;
    virtual void setTag(const std::string& key, const std::string& value) = 0;
    virtual void setLog(std::pair<std::string, std::string> fields) = 0;
    virtual void markError(const std::string& description) = 0;
    // Explicitly set the trace as finished. Any calls to setTag, setLog & markError after this call should
    // leave the trace in a valid but unspecified state. Calling context on a finished trace is a valid operation
    virtual void finish() = 0;
    virtual std::string context() const = 0;
};

class IAlicaTraceFactory
{
public:
    virtual std::unique_ptr<IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent = std::nullopt) const = 0;
    virtual ~IAlicaTraceFactory() = default;
};

} // namespace alica
