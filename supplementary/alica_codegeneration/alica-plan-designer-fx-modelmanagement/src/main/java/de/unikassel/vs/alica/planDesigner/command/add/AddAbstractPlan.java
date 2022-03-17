package de.unikassel.vs.alica.planDesigner.command.add;

import de.unikassel.vs.alica.planDesigner.alicamodel.AbstractPlan;
import de.unikassel.vs.alica.planDesigner.alicamodel.ConfAbstractPlanWrapper;
import de.unikassel.vs.alica.planDesigner.alicamodel.State;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class AddAbstractPlan extends Command {
    protected State state;
    protected AbstractPlan abstractPlan;
    protected ConfAbstractPlanWrapper wrapper;

    public AddAbstractPlan(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        state = (State) modelManager.getPlanElement(mmq.getParentId());
        abstractPlan = (AbstractPlan) modelManager.getPlanElement(mmq.getElementId());

        if(modelManager.checkForInclusionLoop(state, abstractPlan)){
            throw new RuntimeException(
                    String.format("AbstractPlan \"%s\" can not be added to State \"%s\" because of loop in model",
                    abstractPlan.getName(), state.getName())
            );
        }

        // wrap abstract plan with configuration remaining null -> no extra configuration
        wrapper = new ConfAbstractPlanWrapper();
        wrapper.setAbstractPlan(this.abstractPlan);

        // change element type, because the abstract plan is wrapped
        mmq.setElementType(Types.CONF_ABSTRACTPLAN_WRAPPER);
    }

    @Override
    public void doCommand() {
        state.addConfAbstractPlanWrapper(wrapper);
        modelManager.storePlanElement(Types.CONF_ABSTRACTPLAN_WRAPPER, wrapper, false);
        fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, wrapper);
    }

    @Override
    public void undoCommand() {
        state.removeConfAbstractPlanWrapper(wrapper);
        modelManager.dropPlanElement(Types.CONF_ABSTRACTPLAN_WRAPPER, wrapper, false);
        fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, wrapper);
    }
}
