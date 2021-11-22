#include "PlanAttachmentCreator.h"
#include "AttackPlan1402488634525.h"
#include "Authority/AuthorityTest1414403413451.h"
#include "Authority/AuthorityTestMaster1414403396328.h"
#include "BackForth1529456584982.h"
#include "BehaviorSuccessSpamMaster1522377375148.h"
#include "BehaviourTriggerTestPlan1428508768572.h"
#include "Configurations/ConfigurationTestPlan1588060981661.h"
#include "Configurations/ReadConfInPlantype1588061801734.h"
#include "Configurations/ReadConfigurationPlan1588061334567.h"
#include "ConstraintTestMaster1414068495566.h"
#include "ConstraintTestPlan1414068524245.h"
#include "Defend1402488893641.h"
#include "EngineRulesSchedulingTestMaster1625610679488.h"
#include "EngineRulesSchedulingTestPlan1625614640417.h"
#include "FailsOnOne1530069246103.h"
#include "FrequencyTestPlan1626848999740.h"
#include "GoalPlan1402488870347.h"
#include "HandleFailExplicit1530004915640.h"
#include "HandleFailExplicitMaster1530004940652.h"
#include "MasterPlan1402488437260.h"
#include "MasterPlanTaskAssignment1407152758497.h"
#include "MasterPlanTestConditionPlanType1418042656594.h"
#include "MasterSyncTransition1418825395939.h"
#include "MidFieldPlayPlan1402488770050.h"
#include "MultiAgentTestMaster1413200842973.h"
#include "MultiAgentTestPlan1413200862180.h"
#include "OrderedSchedulingTestPlan1629895582410.h"
#include "OtherPlan1418042819203.h"
#include "PlanA1629895837159.h"
#include "PlanAA1629895864090.h"
#include "PlanB1629895853508.h"
#include "PlanBA1629895873188.h"
#include "PlanFive1407153703092.h"
#include "PlanFour1407153683051.h"
#include "PlanOne1407153611768.h"
#include "PlanPoolTestMasterPlan1964838032551226161.h"
#include "PlanPoolTestSubPlan432995127772554364.h"
#include "PlanThree1407153663917.h"
#include "PlanTwo1407153645238.h"
#include "PreConditionPlan1418042796751.h"
#include "RealMasterPlanForSyncTest1418902217839.h"
#include "RuntimeConditionPlan1418042806575.h"
#include "SchedulingTestMasterPlan1613378382024.h"
#include "SchedulingTestPlan11613378406860.h"
#include "SchedulingTestPlan21613378423610.h"
#include "SchedulingTestPlan31613378433623.h"
#include "SchedulingTestSequencePlan11614963946725.h"
#include "SchedulingTestSequenceSubPlan11614964379654.h"
#include "SchedulingTestSequenceSubPlan21614964444419.h"
#include "SchedulingTestSequenceSubPlan31614964478264.h"
#include "SimpleTestPlan1412252439925.h"
#include "Tackle1402489318663.h"
#include "engine/PlanAttachment.h"

namespace alica
{

PlanAttachmentCreator::PlanAttachmentCreator() {}
PlanAttachmentCreator::~PlanAttachmentCreator() {}

class DefaultPlanAttachment : public PlanAttachment
{
    bool setParameters(const BlackBoard& parent_bb, BlackBoard& child_bb){return true;}
};

std::unique_ptr<PlanAttachment> PlanAttachmentCreator::createPlanAttachment(int64_t attachmentWrapperConfId)
{
    switch (attachmentWrapperConfId) {
    default:
        return std::make_unique<DefaultPlanAttachment>();
    }
}

} // namespace alica
