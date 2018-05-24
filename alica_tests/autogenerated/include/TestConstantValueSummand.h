/*
 * TestConstantValueSummand.h
 *
 *  Created on: Dec 11, 2014
 *      Author: Paul Panin
 */

#ifndef TESTCONSTANTVALUESUMMAND_H_
#define TESTCONSTANTVALUESUMMAND_H_

#include <engine/USummand.h>

namespace supplementary
{
class AgentID;
}

namespace alica
{

class TestConstantValueSummand : public USummand
{
  public:
    TestConstantValueSummand(double weight, std::string name, long id, double val);
    virtual ~TestConstantValueSummand();
    void cacheEvalData();
    UtilityInterval eval(IAssignment* ass);
    const supplementary::AgentID* robotId;

  protected:
    double val;
};

} /* namespace alica */

#endif /* TESTCONSTANTVALUESUMMAND_H_ */
