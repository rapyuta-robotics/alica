package de.unikassel.vs.alica.planDesigner.events;

import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiElement;

import java.util.Map;

public class ModelEvent {
    protected ModelEventType eventType;
    protected String elementType;
    protected PlanElement element;
    protected long parentId;

    protected String changedAttribute;
    protected Object newValue;
    protected Object oldValue;

    protected Map<String, Long> relatedObjects;
    protected UiElement uiElement;

    public ModelEvent(ModelEventType eventType, PlanElement element, String elementType) {
        this.eventType = eventType;
        this.element = element;
        this.elementType = elementType;
    }

    public ModelEventType getEventType() {
        return eventType;
    }

    public PlanElement getElement() {
        return element;
    }

    public String getElementType() {
        return elementType;
    }

    public String getChangedAttribute() {
        return changedAttribute;
    }

    public void setChangedAttribute(String changedAttribute) {
        this.changedAttribute = changedAttribute;
    }

    public long getParentId() {
        return parentId;
    }

    public void setParentId(long parentId) {
        this.parentId = parentId;
    }

    public void setNewValue(Object newValue) {
        this.newValue = newValue;
    }

    public Object getNewValue() {
        return this.newValue;
    }

    public void setOldValue(Object oldValue) {
        this.oldValue = oldValue;
    }

    public Object getOldValue() {
        return this.oldValue;
    }

    public Map<String, Long> getRelatedObjects() {
        return relatedObjects;
    }

    public void setRelatedObjects(Map<String, Long> relatedObjects) {
        this.relatedObjects = relatedObjects;
    }

    public void setUiElement(UiElement uiElement) {
        this.uiElement = uiElement;
    }

    public UiElement getUiElement() {
        return uiElement;
    }
}
