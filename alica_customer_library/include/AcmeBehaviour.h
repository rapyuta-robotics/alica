#pragma once

#include "engine/BasicBehaviour.h"

namespace alica
{
class AcmeBehaviour : public BasicBehaviour
{
public:
    AcmeBehaviour(BehaviourContext& context);
    virtual ~AcmeBehaviour(){};
    void run(void* msg) override{};

    BEHAVIOURREGISTER_DEC_TYPE(AcmeBehaviour);
};
}; // namespace alica
