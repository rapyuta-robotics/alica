package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.command.ConditionCommand;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;

public class DeleteCondition extends ConditionCommand {

    protected Condition oldCondition;
    protected PlanElement planElement;

    public DeleteCondition(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.planElement = modelManager.getPlanElement(mmq.getParentId());
        this.oldCondition = (Condition) modelManager.getPlanElement(mmq.getElementId());
    }

    @Override
    public void doCommand() {
        setCondition(null, planElement);

        modelManager.dropPlanElement(mmq.getElementType(), oldCondition, false);
        fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, oldCondition);
    }

    @Override
    public void undoCommand() {
        setCondition(oldCondition, planElement);

        modelManager.storePlanElement(mmq.getElementType(), oldCondition, false);
        fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, oldCondition);
    }
}
