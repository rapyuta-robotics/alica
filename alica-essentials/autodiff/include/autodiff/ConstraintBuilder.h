#pragma once
#include "Types.h"
namespace autodiff
{

namespace Constraints
{
TermPtr equals(TermPtr t1, TermPtr t2, TermPtr tolerance);
TermPtr ifThen(TermPtr tif, TermPtr tthen);
TermPtr ifThenElse(TermPtr tif, TermPtr tthen, TermPtr telse);
TermPtr equiv(TermPtr a, TermPtr b);
TermPtr applyConstraint(TermPtr constraint, TermPtr utility);
}

} // namespace autodiff
