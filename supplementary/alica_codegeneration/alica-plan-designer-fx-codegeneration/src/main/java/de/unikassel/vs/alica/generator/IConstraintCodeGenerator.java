package de.unikassel.vs.alica.generator;

import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.Behaviour;
import de.unikassel.vs.alica.planDesigner.alicamodel.State;

/**
 * This interface defines the methods that a constraint plugin must implement.
 */
public interface IConstraintCodeGenerator {
    String constraintPlanCheckingMethods(Plan plan);
    String constraintBehaviourCheckingMethods(Behaviour behaviour);
    String expressionsPlanCheckingMethods(Plan plan);
    String expressionsBehaviourCheckingMethods(Behaviour behaviour);
    String constraintStateCheckingMethods(State state);
    String expressionsStateCheckingMethods(State state);
}
