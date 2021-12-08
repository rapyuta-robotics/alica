#pragma once

#include <engine/IAlicaTrace.h>
#include <string>
#include <unordered_map>
#include <optional>

namespace alicaTestTracing
{
class AlicaTestTrace : public alica::IAlicaTrace
{
public:
    AlicaTestTrace(const std::string& opName, std::optional<const std::string> parent);

    void setTag(const std::string& key, const std::string& value);
    void setLog(std::pair<std::string, std::string> logEntry);
    void markError(const std::string& description);
    std::string context() const;

    std::string _opName;
    std::string _parent;
};

class AlicaTestTraceFactory : public alica::IAlicaTraceFactory
{
public:
    AlicaTestTraceFactory() {};
    ~AlicaTestTraceFactory() = default;

    std::unique_ptr<alica::IAlicaTrace> create(const std::string& opName, std::optional<const std::string> parent) const;
};
} // namespace alicaDummyTracing
