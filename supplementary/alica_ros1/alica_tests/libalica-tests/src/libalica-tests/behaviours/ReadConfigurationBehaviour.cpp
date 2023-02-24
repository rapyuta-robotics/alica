#include <libalica-tests/behaviours/ReadConfigurationBehaviour.h>
#include <memory>

#include "engine/model/Parameter.h"

namespace alica
{

ReadConfigurationBehaviour::ReadConfigurationBehaviour(BehaviourContext& context)
        : DomainBehaviour(context)
{
}
ReadConfigurationBehaviour::~ReadConfigurationBehaviour() {}
void ReadConfigurationBehaviour::run() {}
void ReadConfigurationBehaviour::initialiseParameters()
{
    testValue = "1";
}
std::unique_ptr<ReadConfigurationBehaviour> ReadConfigurationBehaviour::create(alica::BehaviourContext& context)
{
    return std::make_unique<ReadConfigurationBehaviour>(context);
}
} /* namespace alica */
