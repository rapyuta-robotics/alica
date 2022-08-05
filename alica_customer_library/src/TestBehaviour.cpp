#include "TestBehaviour.h"

#include <map>
namespace alica
{
TestBehaviour::TestBehaviour(BehaviourContext& context):BasicBehaviour(context) {}
} // namespace alica

extern "C" void Start(char*, char*){};
extern "C" void Stop(char*, char*){};
extern "C" void Configure(const std::map<std::string, std::string>& conf){};
