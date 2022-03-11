package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.PlanType;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CreatePlanType extends Command {
    private PlanType planType;

    public CreatePlanType(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.planType = createPlanType();
    }

    protected PlanType createPlanType() {
        PlanType planType = new PlanType();
        planType.setName(mmq.getName());
        planType.setRelativeDirectory(modelManager.makeRelativeDirectory(mmq.getAbsoluteDirectory(),planType.getName()+"."+Extensions.PLANTYPE));
        planType.registerDirtyFlag();
        return planType;
    }

    @Override
    public void doCommand() {
        this.modelManager.storePlanElement(Types.PLANTYPE, this.planType, true);
        this.fireEvent(ModelEventType.ELEMENT_CREATED, this.planType);
    }

    @Override
    public void undoCommand() {
        modelManager.dropPlanElement(Types.PLANTYPE, this.planType, true);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, this.planType);
    }
}
