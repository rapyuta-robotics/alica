#pragma once

#include <optional>
#include <memory>

namespace alica
{

class IAlicaTrace
{
public:
    virtual ~IAlicaTrace() = default;
    IAlicaTrace(const std::string& opName, std::optional<const std::string> parent = std::nullopt);
    IAlicaTrace(IAlicaTrace&& other) = default;
    IAlicaTrace& operator=(IAlicaTrace&& other) = default;

    virtual void setTag(const std::string& key, const std::string& value) = 0;
    virtual void setLog(std::pair<std::string, std::string> fields) = 0;
    virtual void markError(const std::string& description) = 0;
    virtual std::string context() const = 0;

private:
    std::unique_ptr<IAlicaTrace> _rawTrace;
};

class IAlicaTraceFactory
{
public:
    virtual std::unique_ptr<IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent = std::nullopt) const;
    virtual ~IAlicaTraceFactory() = default;
};

}
