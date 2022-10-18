#pragma once
#include <engine/IBehaviourCreator.h>

#include <functional>
#include <memory>

namespace alica
{

class BasicBehaviour;

class DynamicBehaviourCreator : public IBehaviourCreator
{
public:
    virtual ~DynamicBehaviourCreator();
    std::unique_ptr<BasicBehaviour> createBehaviour(int64_t behaviourId, BehaviourContext& context) override;

private:
    typedef std::unique_ptr<BasicBehaviour>(behaviourCreatorType)(BehaviourContext&);
    std::function<behaviourCreatorType> _behaviourCreator;
};

} /* namespace alica */
