#include "engine/expressionhandler/ExpressionHandler.h"
#include "engine/AlicaContext.h"
#include "engine/AlicaEngine.h"
#include "engine/BasicUtilityFunction.h"
#include "engine/IConditionCreator.h"
#include "engine/IConstraintCreator.h"
#include "engine/IUtilityCreator.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/expressionhandler/BasicFalseCondition.h"
#include "engine/expressionhandler/BasicTrueCondition.h"
#include "engine/expressionhandler/DummyConstraint.h"
#include "engine/model/Condition.h"
#include "engine/model/Plan.h"
#include "engine/model/PreCondition.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/modelmanagement/factories/Factory.h"

namespace alica
{

/**
 * Constructor, loads the assembly containing expressions and constraints.
 */
ExpressionHandler::ExpressionHandler(ConfigChangeListener& configChangeListener)
{
    auto reloadFunctionPtr = std::bind(&ExpressionHandler::reload, this, std::placeholders::_1);
    configChangeListener.subscribe(reloadFunctionPtr);
    reload(configChangeListener.getConfig());
}

ExpressionHandler::~ExpressionHandler() {}

void ExpressionHandler::reload(const YAML::Node& config) {}
/**
 * Attaches expressions and constraints to the plans. Called by the AlicaEngine during start up.
 */
void ExpressionHandler::attachAll(PlanRepository& pr, AlicaCreators& creatorCtx)
{
    for (const std::pair<const int64_t, Plan*>& it : pr._plans) {
        Plan* p = it.second;

        auto ufGen = creatorCtx.utilityCreator->createUtility(p->getId());
        p->_utilityFunction = ufGen->getUtilityFunction(p);

        if (p->getPreCondition() != nullptr) {
            if (p->getPreCondition()->isEnabled()) {
                ConditionContext context{p->getPreCondition()->getName(), p->getPreCondition()->_libraryName, p->getPreCondition()->getId()};
                p->_preCondition->setBasicCondition(creatorCtx.conditionCreator->createConditions(context));
                attachConstraint(p->_preCondition, *creatorCtx.constraintCreator);
            } else {
                p->_preCondition->setBasicCondition(make_shared<BasicFalseCondition>());
            }
        }

        if (p->getRuntimeCondition() != nullptr) {
            ConditionContext context{p->getRuntimeCondition()->getName(), p->getRuntimeCondition()->_libraryName, p->getRuntimeCondition()->getId()};
            p->_runtimeCondition->setBasicCondition(creatorCtx.conditionCreator->createConditions(context));
            attachConstraint(p->_runtimeCondition, *creatorCtx.constraintCreator);
        }
    }
}

void ExpressionHandler::attachConstraint(Condition* c, IConstraintCreator& crc)
{
    if (c->getVariables().size() == 0 && c->getQuantifiers().size() == 0) {
        c->setBasicConstraint(make_shared<DummyConstraint>());
    } else {
        c->setBasicConstraint(crc.createConstraint(c->getId()));
    }
}

} /* namespace alica */
