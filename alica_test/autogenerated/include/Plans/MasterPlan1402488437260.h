#ifndef MasterPlan_H_
#define MasterPlan_H_

#include "DomainCondition.h"
#include "engine/BasicUtilityFunction.h"
#include "engine/UtilityFunction.h"
#include "engine/DefaultUtilityFunction.h"
/*PROTECTED REGION ID(incl1402488437260) ENABLED START*/
//Add inlcudes here
/*PROTECTED REGION END*/
using namespace alica;

namespace alicaAutogenerated
{
/*PROTECTED REGION ID(meth1402488437260) ENABLED START*/
//Add other things here
/*PROTECTED REGION END*/
class UtilityFunction1402488437260 : public BasicUtilityFunction
{
  shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

class TransitionCondition1402488519140 : public DomainCondition
{
  bool evaluate(shared_ptr<RunningPlan> rp);
};

class TransitionCondition1409218319990 : public DomainCondition
{
  bool evaluate(shared_ptr<RunningPlan> rp);
};

class TransitionCondition1402488558741 : public DomainCondition
{
  bool evaluate(shared_ptr<RunningPlan> rp);
};

class TransitionCondition1402488520968 : public DomainCondition
{
  bool evaluate(shared_ptr<RunningPlan> rp);
};

} /* namespace alica */

#endif
