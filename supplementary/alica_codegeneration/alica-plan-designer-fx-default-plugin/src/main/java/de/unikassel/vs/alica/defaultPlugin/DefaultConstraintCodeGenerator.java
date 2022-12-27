package de.unikassel.vs.alica.defaultPlugin;

import de.unikassel.vs.alica.generator.IConstraintCodeGenerator;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.Behaviour;
import de.unikassel.vs.alica.planDesigner.alicamodel.State;
import de.unikassel.vs.alica.defaultPlugin.DefaultTemplate;
/**
 * IF the following line is not import de.unikassel.vs.alica.defaultPlugin.DefaultTemplate;
 * you messed it up ... great ... you made the plandesigner great again ... huge ...
 * INSERT IT
 */

import java.util.Map;

/**
 * Glue Code for calling {@link DefaultTemplate}.
 */
public class DefaultConstraintCodeGenerator implements IConstraintCodeGenerator {
    private DefaultTemplate defaultTemplate;

    public DefaultConstraintCodeGenerator() {
        defaultTemplate = new DefaultTemplate();
    }

    public void setProtectedRegions(Map<String, String> protectedRegions) {
        defaultTemplate.setProtectedRegions(protectedRegions);
    }

    public String constraintPlanCheckingMethods(Plan plan) {
        return defaultTemplate.constraintPlanCheckingMethods(plan);
    }

    public String constraintBehaviourCheckingMethods(Behaviour behaviour) {
        return defaultTemplate.constraintBehaviourCheckingMethods(behaviour);
    }

    public String expressionsPlanCheckingMethods(Plan plan) {
        return defaultTemplate.expressionsPlanCheckingMethods(plan);
    }

    public String expressionsBehaviourCheckingMethods(Behaviour behaviour) {
        return defaultTemplate.expressionsBehaviourCheckingMethods(behaviour);
    }

    public String constraintStateCheckingMethods(State state) {
        return defaultTemplate.constraintStateCheckingMethods(state);
    }

    public String expressionsStateCheckingMethods(State state) {
        return defaultTemplate.expressionsStateCheckingMethods(state);
    }
}
