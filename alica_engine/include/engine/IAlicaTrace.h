#pragma once

#include <amr_interfaces/TraceContext.h>

namespace alica
{

class IAlicaTrace
{
public:
    virtual ~IAlicaTrace() = default;
    IAlicaTrace(const std::string& opName, std::optional<const amr_interfaces::TraceContext> parent = std::nullopt);
    IAlicaTrace(IAlicaTrace&& other) = default;
    IAlicaTrace& operator=(IAlicaTrace&& other) = default;

//    void setTag(const std::string& key, const RawTraceValue& value);
//    void setLog(std::pair<std::string, RawTraceValue> fields);
    void markError(const std::string& description);
    amr_interfaces::TraceContext context() const;

private:
    std::unique_ptr<IAlicaTrace> _rawTrace;
};

class IAlicaTraceFactory
{
public:
    virtual IAlicaTrace create(const std::string& opName, std::optional<const amr_interfaces::TraceContext> parent = std::nullopt) const;
    virtual ~IAlicaTraceFactory() = default;
};

}
