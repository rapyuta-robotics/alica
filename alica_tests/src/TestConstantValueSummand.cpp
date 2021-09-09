/*
 * TestConstantValueSummand.cpp
 *
 *  Created on: Dec 11, 2014
 *      Author: Paul Panin
 */

#include <alica_tests/TestConstantValueSummand.h>

#include <engine/planselector/IAssignment.h>

namespace alica
{

TestConstantValueSummand::TestConstantValueSummand(double weight, double val)
        : USummand(weight)
        , robotId(nullptr)
        , val(val)
{
}

TestConstantValueSummand::~TestConstantValueSummand() {}

UtilityInterval TestConstantValueSummand::eval(IAssignment, const Assignment* oldAss) const
{
    return UtilityInterval(val, val);
}
} /* namespace alica */
