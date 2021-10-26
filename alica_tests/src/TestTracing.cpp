#include "alica_tests/TestTracing.h"

#include "alica_tests/TestWorldModel.h"
#include <cassert>

namespace alicaTestTracing
{
AlicaTestTrace::AlicaTestTrace(const std::string& opName, std::optional<const std::string> parent)
        : _opName(opName)
{
    if (parent) {
        _parent = *parent;
    }
}

void AlicaTestTrace::setTag(const std::string& key, const std::string& value)
{
    alicaTests::TestWorldModel::getOne()->tracingTags.push_back({key, value});
}

void AlicaTestTrace::setLog(std::pair<std::string, std::string> logEntry)
{
    alicaTests::TestWorldModel::getOne()->tracingLogs.push_back({logEntry.first, logEntry.second});
}

void AlicaTestTrace::markError(const std::string& description)
{
    alicaTests::TestWorldModel::getOne()->tracingTags.push_back({"error", "true"});
    alicaTests::TestWorldModel::getOne()->tracingTags.push_back({"error.description", description});
}

std::string AlicaTestTrace::context() const
{
    return _opName;
}

std::unique_ptr<alica::IAlicaTrace> AlicaTestTraceFactory::create(const std::string& opName, std::optional<const std::string> parent) const
{
    assert(!opName.empty());
    std::unique_ptr<AlicaTestTrace> trace = std::make_unique<AlicaTestTrace>(opName, parent);
    return trace;
}
} // namespace alicaTestTracing
