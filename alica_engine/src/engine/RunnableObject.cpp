#include "engine/RunnableObject.h"
#include "engine/AlicaEngine.h"
#include <alica_common_config/debug_output.h>

#include <assert.h>
#include <iostream>

namespace alica
{
RunnableObject::RunnableObject()
        : _engine(nullptr)
        , _configuration(nullptr)
        , _flags(static_cast<uint8_t>(Flags::TRACING_ENABLED))
{
}

void RunnableObject::setEngine(AlicaEngine* engine)
{
    _engine = engine;
}

void RunnableObject::setConfiguration(const Configuration* conf)
{
    _configuration = conf;
}

void RunnableObject::sendLogMessage(int level, const std::string& message) const
{
    _engine->getCommunicator().sendLogMessage(level, message);
}

std::optional<IAlicaTrace*> RunnableObject::getTrace() const
{
    return _trace ? std::optional<IAlicaTrace*>(_trace.get()) : std::nullopt;
}

std::optional<std::string> RunnableObject::getTraceContext() const
{
    return _trace ? std::optional<std::string>(_trace->context()) : std::nullopt;
}

} /* namespace alica */
