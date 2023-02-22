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
    std::uniform_real_distribution<> dist(bb.get<double>("min"), bb.get<double>("max"));
    bb.set<double>("value", dist(engine));
    setSuccess();
}

std::unique_ptr<GenerateRandom> GenerateRandom::create(alica::BehaviourContext& context)
{
    return std::make_unique<GenerateRandom>(context);
}

} // namespace alica_standard_library
