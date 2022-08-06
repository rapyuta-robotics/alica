#pragma once

#include "engine/BasicBehaviour.h"

namespace alica
{
class AcmeBehaviour : public BasicBehaviour
{
public:
    AcmeBehaviour(BehaviourContext& context);
    void run(void* msg) override{};


    //BEHAVIOURREGISTER_DEC_TYPE(AcmeBehaviour)    
    static DerivedBehaviourRegister<AcmeBehaviour> reg_;
};
}; // namespace alica
