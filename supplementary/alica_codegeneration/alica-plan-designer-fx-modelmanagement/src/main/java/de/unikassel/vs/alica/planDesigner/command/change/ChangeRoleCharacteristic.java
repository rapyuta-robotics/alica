package de.unikassel.vs.alica.planDesigner.command.change;

import de.unikassel.vs.alica.planDesigner.alicamodel.Characteristic;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.alicamodel.Role;
import de.unikassel.vs.alica.planDesigner.alicamodel.Task;
import de.unikassel.vs.alica.planDesigner.command.ChangeAttributeCommand;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEvent;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import javafx.collections.ObservableList;
import javafx.util.Pair;
import org.apache.commons.beanutils.PropertyUtils;

import java.lang.reflect.InvocationTargetException;
import java.util.Map;

public class ChangeRoleCharacteristic extends ChangeAttributeCommand {

    private final String elementType;
    private final Role role;
    private final String attribute;
    private final Map<String, Long> relatedObjects;
    private final Characteristic characteristic;
    private final String newValue;
    private final String oldValue;

    public ChangeRoleCharacteristic(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.elementType = mmq.getElementType();
        this.characteristic = (Characteristic) modelManager.getPlanElement(mmq.getElementId());
        this.role = (Role) modelManager.getPlanElement(mmq.getParentId());
        this.attribute = mmq.getAttributeName();
        this.newValue = String.valueOf(mmq.getNewValue());
        this.oldValue = characteristic.getName();
        this.relatedObjects = mmq.getRelatedObjects();
    }

    @Override
    public void doCommand() {
        this.modelManager.changeAttribute(this.characteristic, elementType, attribute, newValue, oldValue);
//        this.fireEvent(this.role, this.elementType, this.attribute);
        this.fireEvent(this.characteristic, this.elementType, this.attribute, this.newValue);
    }

    protected void fireEvent(PlanElement planElement, String elementType, String changedAttribute, String newValue) {

        ModelEvent event = new ModelEvent(ModelEventType.ELEMENT_ATTRIBUTE_CHANGED, planElement, elementType);
        event.setChangedAttribute(changedAttribute);
        event.setNewValue(newValue);

        try {
            // Using PropertyUtils instead of BeanUtils to get the actual Object and not just its String-representation
            event.setNewValue(PropertyUtils.getProperty(planElement, changedAttribute));
        } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
            e.printStackTrace();
        }
        event.setNewValue(newValue);
        this.modelManager.fireEvent(event);
    }

    @Override
    public void undoCommand() {
        this.modelManager.changeAttribute(this.role, elementType, attribute, newValue, oldValue);
        this.fireEvent(this.role, this.elementType, this.attribute);
    }
}
