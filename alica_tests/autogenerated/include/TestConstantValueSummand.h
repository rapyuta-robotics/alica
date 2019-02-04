/*
 * TestConstantValueSummand.h
 *
 *  Created on: Dec 11, 2014
 *      Author: Paul Panin
 */

#ifndef TESTCONSTANTVALUESUMMAND_H_
#define TESTCONSTANTVALUESUMMAND_H_

#include "engine/AgentIDConstPtr.h"
#include <engine/USummand.h>

namespace alica
{

class TestConstantValueSummand : public USummand
{
public:
    TestConstantValueSummand(double weight, double val);
    virtual ~TestConstantValueSummand();

    UtilityInterval eval(IAssignment ass) const override;
    AgentIDConstPtr robotId;

protected:
    double val;
};

} /* namespace alica */

#endif /* TESTCONSTANTVALUESUMMAND_H_ */
