#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class IdleBehaviour : public BasicBehaviour
{
public:
    explicit IdleBehaviour(BehaviourContext& context);
    virtual ~IdleBehaviour(){};
    void run(void* msg) override;

    // Factory method
    static std::unique_ptr<IdleBehaviour> create(BehaviourContext& context) { return std::make_unique<IdleBehaviour>(context); }
};
BOOST_DLL_ALIAS(alica::IdleBehaviour::create, idlebehaviour)
} // namespace alica
