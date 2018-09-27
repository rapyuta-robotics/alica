
#include "ConstraintBuilder.h"

#include "ConstraintUtility.h"
#include "Term.h"
#include "TermHolder.h"
#include "TermPtr.h"
#include <limits>

namespace autodiff
{
namespace Constraints
{
TermPtr equals(TermPtr t1, TermPtr t2, TermPtr tolerance)
{
    return (t1 < (t2 + tolerance)) & (t1 > (t2 - tolerance));
}

TermPtr ifThen(TermPtr tif, TermPtr tthen)
{
    return tif->negate() | tthen;
}

TermPtr ifThenElse(TermPtr tif, TermPtr tthen, TermPtr telse)
{
    return (tif->negate() | tthen) & (tif | telse);
}

TermPtr equiv(TermPtr a, TermPtr b)
{
    return (a & b) | (a->negate() & b->negate());
}

/**
 * Combines a constraint and a utility. This is usually only used by the CGSolver.
 * Use if you need to circumvent CGSolver and work directly with GSolver.
 *
 * @param constraint A Term
 * @param utility A Term
 *
 * @result A Term
 */
TermPtr applyConstraint(TermPtr constraint, TermPtr utility)
{
    return constraint->getOwner()->constraintUtility(constraint, utility);
}
}
} /* namespace autodiff */
