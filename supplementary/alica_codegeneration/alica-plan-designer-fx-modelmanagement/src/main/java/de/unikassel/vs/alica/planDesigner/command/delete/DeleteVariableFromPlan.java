package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.AbstractPlan;
import de.unikassel.vs.alica.planDesigner.alicamodel.Variable;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class DeleteVariableFromPlan extends Command {

    protected Variable variable;
    protected AbstractPlan abstractPlan;

    public DeleteVariableFromPlan(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.variable = (Variable) modelManager.getPlanElement(mmq.getElementId());
        this.abstractPlan = (AbstractPlan) modelManager.getPlanElement(mmq.getParentId());
    }

    @Override
    public void doCommand() {
        this.abstractPlan.removeVariable(this.variable);
        this.modelManager.dropPlanElement(Types.VARIABLE, this.variable, false);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, this.variable);
    }

    @Override
    public void undoCommand() {
        this.abstractPlan.addVariable(this.variable);
        this.modelManager.storePlanElement(Types.VARIABLE, this.variable, false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, this.variable);
    }
}
