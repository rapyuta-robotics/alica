#include "Move.h"

namespace alica
{
// Plan:  Move (1889749086610694100)
//
// Tasks:
//   - Follower (3759439551323513525) (Entrypoint: 3277312192440194145)//   - Leader (826983480584534597) (Entrypoint: 4346694000146342467)
//
// States:
//   - AlignCircle (2299237921449867536)
//   - Move2Center (4158797811607100614)
Move::Move(PlanContext& context)
        : BasicPlan(context)
{
    std::cerr << "Move created" << std::endl;
}
Move::~Move() {}

} // namespace alica
