#pragma once

#include "engine/IAlicaTrace.h"

#include <atomic>
#include <memory>
#include <string>

namespace alica
{
namespace test
{
class TestContext;
}
class Configuration;
class AlicaEngine;

/**
 * The base class for BasicBehaviour, BehaviourPool, BasicPlan, PlanPool
 */
class RunnableObject
{
public:
    RunnableObject();
    virtual ~RunnableObject() = default;
    void setEngine(AlicaEngine* engine);
    void setConfiguration(const Configuration* conf);

    // This is not thread safe. Should only be called by the scheduler thread. TODO: make this private
    std::optional<std::string> getTraceContext() const;

protected:
    std::optional<IAlicaTrace*> getTrace() const;

    using Counter = uint64_t;

    void sendLogMessage(int level, const std::string& message) const;

    AlicaEngine* _engine;
    const Configuration* _configuration;
    std::unique_ptr<IAlicaTrace> _trace;

    enum class Flags : uint8_t
    {
        INIT_EXECUTED = 1u,
        TRACING_ENABLED = 1u << 1,
        RUN_TRACED = 1u << 2
    };

    uint8_t _flags;

    void setFlags(Flags flags) { _flags |= static_cast<uint8_t>(flags); }
    void clearFlags(Flags flags) { _flags &= ~static_cast<uint8_t>(flags); }
    bool areFlagsSet(Flags flags) { return (static_cast<uint8_t>(flags) & _flags) == static_cast<uint8_t>(flags); }
};
} /* namespace alica */
