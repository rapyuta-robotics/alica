#include <alica_tests/PreConditionPlan1418042796751.h>
/*PROTECTED REGION ID(eph1418042796751) ENABLED START*/
// Add additional using directives here
#include "engine/USummand.h"
#include <alica_tests/TestConstantValueSummand.h>
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PreConditionPlan (1418042796751)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1418042796753)
//
// States:
//   - PreConditionTest (1418042796752)
PreConditionPlan1418042796751::PreConditionPlan1418042796751(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1418042796751) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PreConditionPlan1418042796751::~PreConditionPlan1418042796751()
{
    /*PROTECTED REGION ID(dcon1418042796751) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of PreCondition - (Name): NewPreCondition, (ConditionString): Test , (Comment) :

/**
 * Available Vars:
 */
bool PreCondition1418042929966::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* wm)
{
    /*PROTECTED REGION ID(1418042929966) ENABLED START*/
    //--> "PreCondition:1418042929966  not implemented";
    //    	return true;
    BlackboardImpl& impl = const_cast<BlackboardImpl&>(wm->impl());
    alicaTests::TestWorldModel* worldModel = impl.getWorldModel<alicaTests::TestWorldModel>();
    return worldModel->isPreCondition1418042929966();
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418042796753
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418042796751::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418042796751) ENABLED START*/
    shared_ptr<UtilityFunction> function = make_shared<UtilityFunction>(0.5, 0.1, plan);
    function->editUtilSummands().emplace_back(new TestConstantValueSummand(0.5, 1.0));
    return function;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1418042796751) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
