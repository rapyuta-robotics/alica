package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.PlanType;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class DeletePlanType extends Command {

    protected PlanType planType;

    public DeletePlanType(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        planType = (PlanType) modelManager.getPlanElement(mmq.getElementId());
    }

    @Override
    public void doCommand() {
        modelManager.dropPlanElement(Types.PLANTYPE, planType, true);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, this.planType);
    }

    @Override
    public void undoCommand() {
        modelManager.storePlanElement(Types.PLANTYPE, planType, true);
        this.fireEvent(ModelEventType.ELEMENT_CREATED, this.planType);
    }
}
