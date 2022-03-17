package de.unikassel.vs.alica.planDesigner.command.remove;

import de.unikassel.vs.alica.planDesigner.alicamodel.Condition;
import de.unikassel.vs.alica.planDesigner.alicamodel.Variable;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;

public class RemoveVariableFromCondition extends Command {
    protected Condition condition;
    protected Variable variable;

    public  RemoveVariableFromCondition(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.condition = (Condition) modelManager.getPlanElement(mmq.getParentId());
        this.variable = (Variable) modelManager.getPlanElement(mmq.getElementId());
    }

    @Override
    public void doCommand() {
        condition.removeVariable(variable);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED, this.variable);
    }

    @Override
    public void undoCommand() {

        condition.addVariable(variable);
        this.fireEvent(ModelEventType.ELEMENT_ADDED, this.variable);
    }
}