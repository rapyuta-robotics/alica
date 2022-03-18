package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.PreCondition;
import de.unikassel.vs.alica.planDesigner.alicamodel.RuntimeCondition;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CreatePlan extends Command {

    protected Plan plan;

    public CreatePlan(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.plan = createPlan();
    }

    protected Plan createPlan() {
        Plan plan = new Plan();
        plan.setName(mmq.getName());
        plan.setRelativeDirectory(modelManager.makeRelativeDirectory(mmq.getAbsoluteDirectory(), plan.getName() + "." + Extensions.PLAN));
        plan.registerDirtyFlag();
        return plan;
    }

    @Override
    public void doCommand() {
        modelManager.storePlanElement(Types.PLAN, this.plan, true);
        this.fireEvent(ModelEventType.ELEMENT_CREATED, this.plan);
    }

    @Override
    public void undoCommand() {
        modelManager.dropPlanElement(Types.PLAN, this.plan, true);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, this.plan);
    }
}
