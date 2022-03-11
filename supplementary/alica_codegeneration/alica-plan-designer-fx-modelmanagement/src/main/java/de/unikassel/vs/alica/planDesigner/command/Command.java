package de.unikassel.vs.alica.planDesigner.command;


import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.events.ModelEvent;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;

public abstract class Command {

    protected ModelManager modelManager;
    protected ModelModificationQuery mmq;

    public abstract void doCommand();
    public abstract void undoCommand();

    public Command(ModelManager modelManager, ModelModificationQuery mmq) {
        this.modelManager = modelManager;
        this.mmq = mmq;
    }

    protected void fireEvent(ModelEventType eventType, PlanElement planElement) {
        ModelEvent event = new ModelEvent(eventType, planElement, mmq.getElementType());
        event.setParentId(mmq.getParentId());
        this.modelManager.fireEvent(event);
    }
}
