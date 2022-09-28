#include "Move.h"

namespace alica
{

Move::Move(PlanContext& context)
        : BasicPlan(context)
{
    std::cerr << "Move created" << std::endl;
}
Move::~Move() {}

} // namespace alica
