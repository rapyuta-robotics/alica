package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.AnnotatedPlan;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanType;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CreateAnnotatedPlan extends Command {

    private PlanType planType;
    private AnnotatedPlan annotatedPlan;

    public CreateAnnotatedPlan(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.planType = (PlanType) modelManager.getPlanElement(mmq.getParentId());
        Plan plan = (Plan) modelManager.getPlanElement(mmq.getElementId());
        if(this.modelManager.checkForInclusionLoop(planType, plan)){
            throw new RuntimeException(
                    String.format("Plan \"%s\" can not be added to PlanType \"%s\" because of loop in model"
                            , plan.getName(), planType.getName()));
        }
        this.annotatedPlan = createAnnotatedPlan();
    }

    public AnnotatedPlan createAnnotatedPlan() {
        AnnotatedPlan annotatedPlan = new AnnotatedPlan();
        annotatedPlan.setPlan((Plan) modelManager.getPlanElement(mmq.getElementId()));
        annotatedPlan.setActivated(true);
        return annotatedPlan;
    }

    @Override
    public void doCommand() {
        this.planType.addAnnotatedPlan(annotatedPlan);
        this.modelManager.storePlanElement(Types.ANNOTATEDPLAN, annotatedPlan, false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, annotatedPlan);
    }

    @Override
    public void undoCommand() {
        planType.removeAnnotatedPlan(annotatedPlan);
        modelManager.dropPlanElement(Types.ANNOTATEDPLAN, annotatedPlan, false);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, annotatedPlan);
    }
}
