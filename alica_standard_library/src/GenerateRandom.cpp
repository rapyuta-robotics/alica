#include "GenerateRandom.h"

#include <random>

namespace alica_standard_library
{

GenerateRandom::GenerateRandom(alica::BehaviourContext& context)
        : BasicBehaviour(context)
{
}

void GenerateRandom::initialiseParameters()
{
    alica::LockedBlackboardRW bb(*(getBlackboard()));
    std::random_device seed_gen;
    std::mt19937 engine(seed_gen());
    alica::Logging::logInfo("GenerateRandom") << "min: " << bb.get<double>("min");
    alica::Logging::logInfo("GenerateRandom") << "max: " << bb.get<double>("max");
    std::uniform_real_distribution<> dist(bb.get<double>("min"), bb.get<double>("max"));
    bb.set<double>("value", dist(engine));
    alica::Logging::logInfo("GenerateRandom") << "Generated random num " << bb.get<double>("value");
    setSuccess();
}

std::unique_ptr<GenerateRandom> GenerateRandom::create(alica::BehaviourContext& context)
{
    return std::make_unique<GenerateRandom>(context);
}

} // namespace alica_standard_library
