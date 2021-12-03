#pragma once

#include <optional>
#include <memory>

namespace alica
{

class IAlicaTrace
{
public:
    virtual ~IAlicaTrace() = default;
    virtual void setTag(const std::string& key, const std::string& value) = 0;
    virtual void setLog(std::pair<std::string, std::string> fields) = 0;
    virtual void markError(const std::string& description) = 0;
    virtual void finish() = 0;
    // Note: the context should be valid even after finish() is called on the trace
    virtual std::string context() const = 0;
};

class IAlicaTraceFactory
{
public:
    virtual std::unique_ptr<IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent = std::nullopt) const = 0;
    virtual ~IAlicaTraceFactory() = default;
};

}
