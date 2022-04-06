package de.unikassel.vs.alica.planDesigner.command.change;

import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.command.UiPositionCommand;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.BendPoint;

import java.util.HashMap;

public class ChangePosition extends UiPositionCommand {

    protected PlanElement planElement;
    protected BendPoint bendPoint;

    protected int oldX;
    protected int oldY;

    public ChangePosition(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.planElement = modelManager.getPlanElement(mmq.getElementId());
        this.uiElement = this.createUiElement(mmq.getParentId(), planElement);

        if(mmq.getElementType() != Types.BENDPOINT) {
            this.oldX = uiElement.getX();
            this.oldY = uiElement.getY();
        } else {
            this.planElement = modelManager.getPlanElement(mmq.getElementId());
            this.uiElement = this.createUiElement(mmq.getParentId(), planElement);

           for(BendPoint bPoint : uiElement.getBendPoints()){
                if(bPoint.getId() == mmq.getRelatedObjects().get(Types.BENDPOINT)){
                    bendPoint = bPoint;
                    this.oldX = bendPoint.getX();
                    this.oldY = bendPoint.getY();
                }
            }
        }
    }

    @Override
    public void doCommand() {
        if(bendPoint == null) {
            uiElement.setX(x);
            uiElement.setY(y);
            this.fireEvent(ModelEventType.ELEMENT_CHANGED_POSITION, this.planElement);
        } else {
            bendPoint.setX(x);
            bendPoint.setY(y);
            HashMap<String, Long> bendPointMap = new HashMap<>();
            bendPointMap.put(Types.BENDPOINT, bendPoint.getId());
            this.fireEvent(ModelEventType.ELEMENT_CHANGED_POSITION, this.planElement, bendPointMap);
        }
    }

    @Override
    public void undoCommand() {
        if(bendPoint == null) {
            uiElement.setX(oldX);
            uiElement.setY(oldY);
            this.fireEvent(ModelEventType.ELEMENT_CHANGED_POSITION, this.planElement);
        } else {
            bendPoint.setX(oldX);
            bendPoint.setY(oldY);
            HashMap<String, Long> bendPointMap = new HashMap<>();
            bendPointMap.put(Types.BENDPOINT, bendPoint.getId());
            this.fireEvent(ModelEventType.ELEMENT_CHANGED_POSITION, this.planElement, bendPointMap);
        }
    }
}
