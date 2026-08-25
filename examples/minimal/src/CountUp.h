#pragma once

#include <engine/BasicBehaviour.h>

namespace minimal
{

/**
 * Domain-specific code. ALICA treats this as an atomic black box that can
 * succeed or fail; all it knows is when to start, tick and stop it.
 *
 * This one bumps a counter on the global blackboard on every tick and
 * reports success once it has counted far enough.
 */
class CountUp : public alica::BasicBehaviour
{
public:
    explicit CountUp(alica::BehaviourContext& context);

    static std::unique_ptr<CountUp> create(alica::BehaviourContext& context);

    /** Key on the global blackboard shared with the CountReached condition. */
    static constexpr const char* COUNTER_KEY = "counter";
    static constexpr int64_t TARGET = 6;

protected:
    void initialiseParameters() override;
    void run() override;
    void onTermination() override;
};

} // namespace minimal
