#pragma once

namespace alica
{

class IAlicaTrace
{
public:
    virtual ~IAlicaTrace() = default;
//    IAlicaTrace(const std::string& opName, std::optional<const amr_interfaces::TraceContext> parent = std::nullopt);
    IAlicaTrace(const std::string& opName)
    IAlicaTrace(IAlicaTrace&& other) = default;
    Trace& operator=(Trace&& other) = default;
    virtual ~Trace();

    void setTag(const std::string& key, const RawTraceValue& value);
    void setLog(std::pair<std::string, RawTraceValue> fields);
    void markError(const std::string& description);
    amr_interfaces::TraceContext context() const;

private:
    RawTracePtr _rawTrace;
};

class IAlicaTraceFactory
{
public:
//    virtual IAlicaTrace create(const std::string& opName, std::optional<const amr_interfaces::TraceContext> parent = std::nullopt) const;
    virtual IAlicaTrace create(const std::string& opName) const;
    virtual ~IAlicaTraceFactory() = default;
};

}
