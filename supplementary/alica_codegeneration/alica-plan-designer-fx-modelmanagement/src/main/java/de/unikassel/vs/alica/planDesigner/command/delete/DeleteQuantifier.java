package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.Condition;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.alicamodel.Quantifier;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class DeleteQuantifier extends Command {

    protected Quantifier quantifier;
    protected PlanElement parentElement;

    public DeleteQuantifier(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.quantifier = (Quantifier) this.modelManager.getPlanElement(this.mmq.getElementId());
        this.parentElement = this.modelManager.getPlanElement(this.mmq.getParentId());
    }

    @Override
    public void doCommand() {
        ((Condition) this.parentElement).removeQuantifier(this.quantifier);
        this.modelManager.dropPlanElement(Types.QUANTIFIER, this.quantifier, false);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, this.quantifier);
    }

    @Override
    public void undoCommand() {
        ((Condition) parentElement).addQuantifier(this.quantifier);
        this.modelManager.storePlanElement(Types.QUANTIFIER, this.quantifier, false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, this.quantifier);
    }
}
