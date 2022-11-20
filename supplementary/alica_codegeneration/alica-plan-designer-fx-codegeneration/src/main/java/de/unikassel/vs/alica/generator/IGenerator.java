package de.unikassel.vs.alica.generator;

import de.unikassel.vs.alica.planDesigner.alicamodel.Behaviour;
import de.unikassel.vs.alica.planDesigner.alicamodel.Condition;
import de.unikassel.vs.alica.planDesigner.alicamodel.TransitionCondition;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;

import java.util.List;
import java.util.Map;

/**
 * This interface declares methods for generating code.
 * An example for an implementation for C++ as target language
 * can be found at {@link de.unikassel.vs.alica.generator.cpp.CPPGeneratorImpl}
 */
public interface IGenerator {

    void setProtectedRegions(Map<String, String> protectedRegions);
    void createBehaviourCreator(List<Behaviour> behaviours);
    void createBehaviour(Behaviour behaviour);
    void createConditionCreator(List<Plan> plans, List<Behaviour> behaviours, List<Condition> conditions);
    void createConstraintCreator(List<Plan> plans, List<Behaviour> behaviours, List<Condition> conditions);
    void createConstraints(List<Plan> plans);
    void createConstraintsForPlan(Plan plan);
    void createConstraintsForBehaviour(Behaviour behaviour);
    void createPlans(List<Plan> plans);
    void createPlan(Plan plan);
    void createTransitionConditions(List<TransitionCondition> conditions);
    void createUtilityFunctionCreator(List<Plan> plans);

    void createDomainCondition();
    void createDomainBehaviour();

    void setFormatter(String formatter);
    IConstraintCodeGenerator getActiveConstraintCodeGenerator();

    void createPlanCreator(List<Plan> plans);
    void createTransitionConditionsCreator(List<TransitionCondition> conditions);
    void createLegacyTransitionConditionsCreator(List<Plan> plans, List<Condition> conditions);

    void createDomainPlan();
}
