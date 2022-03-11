package de.unikassel.vs.alica.planDesigner.command;

import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.events.ModelEvent;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiElement;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiExtension;

import java.util.HashMap;

public abstract class UiPositionCommand extends Command {

    protected int x;
    protected int y;
    protected UiExtension uiExtension;
    protected UiElement uiElement;

    public UiPositionCommand(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        x = this.mmq.getX();
        y = this.mmq.getY();
    }

    protected UiElement createUiElement(long planId, PlanElement planElement) {
        this.uiExtension =  this.modelManager.getPlanUIExtensionPair(planId);
        UiElement uiElement = this.uiExtension.getUiElement(planElement.getId());
        uiElement.setX(this.x);
        uiElement.setY(this.y);
        return uiElement;
    }

    @Override
    protected void fireEvent(ModelEventType eventType, PlanElement planElement) {
        ModelEvent event = new ModelEvent(eventType, planElement, mmq.getElementType());
        event.setUiElement(this.uiElement);
        event.setParentId(mmq.getParentId());
        this.modelManager.fireEvent(event);
    }


    protected void fireEvent(ModelEventType eventType, PlanElement planElement, HashMap<String, Long> relatedElement) {
        ModelEvent event = new ModelEvent(eventType, planElement, mmq.getElementType());
        event.setUiElement(this.uiElement);
        event.setParentId(mmq.getParentId());
        event.setRelatedObjects(relatedElement);
        this.modelManager.fireEvent(event);
    }
}

