#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class WaitBehaviour : public BasicBehaviour
{
public:
    explicit WaitBehaviour(BehaviourContext& context);
    virtual ~WaitBehaviour(){};
    void run(void* msg) override;

    // Factory method
    static std::unique_ptr<WaitBehaviour> create(BehaviourContext& context) { return std::make_unique<WaitBehaviour>(context); }
};
BOOST_DLL_ALIAS(alica::WaitBehaviour::create, WaitBehaviour)
} // namespace alica
