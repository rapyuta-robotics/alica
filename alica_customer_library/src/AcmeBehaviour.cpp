#include "AcmeBehaviour.h"

#include <map>
using namespace alica;

//BEHAVIOURREGISTER_DEF_TYPE(AcmeBehaviour, "testbehaviour");
DerivedBehaviourRegister<AcmeBehaviour> AcmeBehaviour::reg_("testbehaviour");

namespace alica
{

AcmeBehaviour::AcmeBehaviour(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
} // namespace alica

extern "C" void Start(char*, char*){};
extern "C" void Stop(char*, char*){};
extern "C" void Configure(const std::map<std::string, std::string>& conf){};
