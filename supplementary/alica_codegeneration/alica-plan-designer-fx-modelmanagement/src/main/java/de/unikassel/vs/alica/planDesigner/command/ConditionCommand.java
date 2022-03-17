package de.unikassel.vs.alica.planDesigner.command;

import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public abstract class ConditionCommand extends Command {
    public ConditionCommand(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
    }

    protected void setCondition(Condition condition, PlanElement planElement) {
        switch (mmq.getElementType()) {
            case Types.PRECONDITION:
                if (planElement instanceof Behaviour) {
                    ((Behaviour) planElement).setPreCondition((PreCondition) condition);
                } else if (planElement instanceof Plan) {
                    ((Plan) planElement).setPreCondition((PreCondition) condition);
                } else if (planElement instanceof Transition) {
                    ((Transition) planElement).setPreCondition((PreCondition) condition);
                } else {
                    throw new RuntimeException("CreateCondition: Element type " + mmq.getElementType() + " does not contain PreConditions!");
                }
                break;
            case Types.RUNTIMECONDITION:
                if (planElement instanceof Behaviour) {
                    ((Behaviour) planElement).setRuntimeCondition((RuntimeCondition) condition);
                } else if (planElement instanceof Plan) {
                    ((Plan) planElement).setRuntimeCondition((RuntimeCondition) condition);
                } else {
                    throw new RuntimeException("CreateCondition: Element type " + mmq.getElementType() + " does not contain RuntimeConditions!");
                }
                break;
            case Types.POSTCONDITION:
                if (planElement instanceof Behaviour) {
                    ((Behaviour) planElement).setPostCondition((PostCondition) condition);
                } else if (planElement instanceof TerminalState) {
                    ((TerminalState) planElement).setPostCondition((PostCondition) condition);
                } else {
                    throw new RuntimeException("CreateCondition: Element type " + mmq.getElementType() + " does not contain PostConditions!");
                }
                break;
            default:
                throw new RuntimeException("CreateCondition: Condition type " + mmq.getElementType() + " does not exist!");
        }
    }
}
