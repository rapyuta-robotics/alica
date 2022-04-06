package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.command.UiPositionCommand;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;


public class CreateTransition extends UiPositionCommand {
    protected State in;
    protected State out;
    protected Transition transition;
    protected Plan plan;

    public CreateTransition(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.plan = (Plan) modelManager.getPlanElement(mmq.getParentId());
        this.in = (State) modelManager.getPlanElement(mmq.getRelatedObjects().get(Types.INSTATE));
        this.out = (State) modelManager.getPlanElement(mmq.getRelatedObjects().get(Types.OUTSTATE));
        this.transition = createTransition();
        this.uiElement = this.createUiElement(plan.getId(), this.transition);
    }

    protected Transition createTransition() {
        Transition transition = new Transition();
        transition.setInState(this.in);
        transition.setOutState(this.out);
        if (mmq.getName() == PlanElement.NO_NAME) {
            transition.setName("From" + this.in.getName() + "To " + this.out.getName());
        } else {
            transition.setName(mmq.getName());
        }
        transition.setComment(mmq.getComment());

        PreCondition preCondition = new PreCondition();
        preCondition.setEnabled(true);
        this.modelManager.storePlanElement(Types.PRECONDITION, preCondition, false);

        transition.setPreCondition(preCondition);

        return transition;
    }

    @Override
    public void doCommand() {
        this.plan.addTransition(this.transition);
        this.in.addOutTransition(this.transition);
        this.out.addInTransition(this.transition);

        this.modelManager.storePlanElement(Types.TRANSITION, this.transition,false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, this.transition);
    }

    @Override
    public void undoCommand() {
        this.plan.removeTransition(this.transition);
        this.in.removeOutTransition(this.transition);
        this.out.removeInTransition(this.transition);
        this.modelManager.dropPlanElement(Types.TRANSITION, this.transition, false);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, this.transition);
    }
}
