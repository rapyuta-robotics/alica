package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CreateVariable extends Command {

    private final AbstractPlan abstractPlan;
    private final Variable variable;

    public CreateVariable(ModelManager manager, ModelModificationQuery mmq) {
        super(manager, mmq);
        this.abstractPlan = (AbstractPlan) modelManager.getPlanElement(mmq.getParentId());
        this.variable = createVariable();
    }

    protected Variable createVariable() {
        Variable variable = new Variable();
        variable.setName(mmq.getName());
        variable.setComment(mmq.getComment());
        return variable;
    }

    @Override
    public void doCommand() {
        abstractPlan.addVariable(this.variable);
        this.modelManager.storePlanElement(Types.VARIABLE, variable, false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, this.variable);
    }

    @Override
    public void undoCommand() {
        abstractPlan.removeVariable(this.variable);
        modelManager.dropPlanElement(Types.VARIABLE, variable,false);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, this.variable);
    }

}
