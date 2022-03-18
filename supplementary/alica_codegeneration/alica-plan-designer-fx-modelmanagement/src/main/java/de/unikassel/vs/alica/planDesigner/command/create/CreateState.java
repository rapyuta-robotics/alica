package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.PostCondition;
import de.unikassel.vs.alica.planDesigner.alicamodel.State;
import de.unikassel.vs.alica.planDesigner.alicamodel.TerminalState;
import de.unikassel.vs.alica.planDesigner.command.UiPositionCommand;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CreateState extends UiPositionCommand {

    protected State state;
    protected Plan plan;

    public CreateState(ModelManager manager, ModelModificationQuery mmq) {
        super(manager, mmq);
        this.plan = (Plan) this.modelManager.getPlanElement(mmq.getParentId());
        this.state = createState();
        this.uiElement = createUiElement(this.plan.getId(), this.state);
    }

    protected State createState() {
        State state;
        if (mmq.getElementType() == Types.STATE) {
            state = new State();
        } else if (mmq.getElementType() == Types.SUCCESSSTATE) {
            state = new TerminalState(true);
        } else if (mmq.getElementType() == Types.FAILURESTATE) {
            state = new TerminalState(false);
        } else {
            throw new RuntimeException("CreateState: Unknown type of state " + mmq.getElementType() + " requested!");
        }
        state.setParentPlan(plan);
        state.setName(mmq.getName());
        state.setComment(mmq.getComment());
        return state;
    }

    @Override
    public void doCommand() {
        this.plan.addState(state);
        this.uiExtension.add(state.getId(), uiElement);
        modelManager.storePlanElement(mmq.getElementType(), state,false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, state);
    }

    @Override
    public void undoCommand() {
        this.plan.removeState(state);
        this.uiExtension.remove(state.getId());
        modelManager.dropPlanElement(mmq.getElementType(), state,false);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, state);
    }
}
