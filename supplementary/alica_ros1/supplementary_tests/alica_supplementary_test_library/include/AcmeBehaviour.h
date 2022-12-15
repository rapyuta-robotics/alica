#pragma once

#include "engine/BasicBehaviour.h"
#include <boost/dll/alias.hpp>

namespace alica
{
class AcmeBehaviour : public BasicBehaviour
{
public:
    AcmeBehaviour(BehaviourContext& context);
    virtual ~AcmeBehaviour(){};
    void run(void* msg) override{};

    // Factory method
    static std::unique_ptr<AcmeBehaviour> create(BehaviourContext& context) { return std::make_unique<AcmeBehaviour>(context); }
};
BOOST_DLL_ALIAS(alica::AcmeBehaviour::create, AcmeBeh)
}; // namespace alica
