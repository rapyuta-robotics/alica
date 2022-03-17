package de.unikassel.vs.alica.planDesigner.command.remove;

import de.unikassel.vs.alica.planDesigner.alicamodel.ConfAbstractPlanWrapper;
import de.unikassel.vs.alica.planDesigner.alicamodel.State;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;

public class RemoveAbstractPlanFromState extends Command {
    protected State state;
    protected ConfAbstractPlanWrapper confAbstractPlanWrapper;

    public RemoveAbstractPlanFromState(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.state = (State) modelManager.getPlanElement(mmq.getParentId());
        this.confAbstractPlanWrapper = (ConfAbstractPlanWrapper) modelManager.getPlanElement(mmq.getElementId());
    }

    @Override
    public void doCommand() {
        this.state.removeConfAbstractPlanWrapper(confAbstractPlanWrapper);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED, confAbstractPlanWrapper);
    }

    @Override
    public void undoCommand() {
        this.state.addConfAbstractPlanWrapper(confAbstractPlanWrapper);
        this.fireEvent(ModelEventType.ELEMENT_ADDED, confAbstractPlanWrapper);
    }
}
