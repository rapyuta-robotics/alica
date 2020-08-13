#pragma once

#include <engine/IBehaviourCreator.h>

#include <unordered_map>

namespace alica
{
class BasicBehaviour;
class TestBehaviourCreator : public IBehaviourCreator
{
public:
    TestBehaviourCreator(IBehaviourCreator& defaultBehaviourCreator);
    virtual ~TestBehaviourCreator() {}
    virtual std::shared_ptr<BasicBehaviour> createBehaviour(int64_t behaviourId);

    /**
     * This method allows to configure for which behaviour a mockup will be
     * returned by the createBehaviour(int64_t behaviourId) method of this class.
     *
     * For all others, the default behaviour creator passed in the constructor will
     * be asked.
     * @param behaviourId
     * @param behaviourMockUp
     */
    void setBehaviourMockUp(int64_t behaviourId, std::shared_ptr<BasicBehaviour> behaviourMockUp);

private:
    std::unordered_map<int64_t, std::shared_ptr<BasicBehaviour>> behaviourMockUps;
    IBehaviourCreator& defaultBehaviourCreator;
};
} // namespace alica
