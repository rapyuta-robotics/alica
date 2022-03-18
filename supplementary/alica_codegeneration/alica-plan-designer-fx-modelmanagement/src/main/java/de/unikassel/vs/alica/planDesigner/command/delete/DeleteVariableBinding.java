package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class DeleteVariableBinding extends Command {

    protected VariableBinding variableBinding;
    protected PlanElement parent;

    public DeleteVariableBinding(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.variableBinding = (VariableBinding) modelManager.getPlanElement(mmq.getElementId());
        this.parent = modelManager.getPlanElement(mmq.getParentId());
    }

    @Override
    public void doCommand() {
        if (parent instanceof State) {
            ((State) this.parent).removeVariableBinding(variableBinding);
        } else if (parent instanceof PlanType) {
            ((PlanType) this.parent).removeVariableBinding(variableBinding);
        }
        this.modelManager.dropPlanElement(Types.VARIABLE, this.variableBinding, false);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, this.variableBinding);
    }

    @Override
    public void undoCommand() {
        if (parent instanceof State) {
            ((State) this.parent).addVariableBinding(variableBinding);
        } else if (parent instanceof PlanType) {
            ((PlanType) this.parent).addVariableBinding(variableBinding);
        }
        this.modelManager.storePlanElement(Types.VARIABLEBINDING, this.variableBinding, false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, this.variableBinding);
    }
}
