#include "AcmeBehaviour.h"

#include <map>
using namespace alica;

BEHAVIOURREGISTER_DEF_TYPE(AcmeBehaviour, "testbehaviour");

namespace alica
{

AcmeBehaviour::AcmeBehaviour(BehaviourContext& context)
        : BasicBehaviour(context)
{
    std::cerr << "AcmeBehaviour created" << std::endl;
}
} // namespace alica

extern "C" void Start(char*, char*){};
extern "C" void Stop(char*, char*){};
extern "C" void Configure(const std::map<std::string, std::string>& conf){};
