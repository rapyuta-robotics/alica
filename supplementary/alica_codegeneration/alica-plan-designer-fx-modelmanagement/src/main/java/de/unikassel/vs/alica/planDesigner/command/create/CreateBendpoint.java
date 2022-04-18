package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.Transition;

import de.unikassel.vs.alica.planDesigner.command.UiPositionCommand;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.BendPoint;

import java.util.HashMap;

public class CreateBendpoint extends UiPositionCommand {

    protected BendPoint bendPoint;
    protected int index = -1;

    public CreateBendpoint(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.bendPoint = createBendPoint();
        this.uiElement = modelManager.getPlanUIExtensionPair(mmq.getParentId()).getUiElement(this.bendPoint.getTransition().getId());
        if(mmq.getRelatedObjects().containsKey("index")){
            this.index = Math.toIntExact(mmq.getRelatedObjects().get("index"));
        }
    }

    protected BendPoint createBendPoint() {
        BendPoint bendPoint = new BendPoint();
        bendPoint.setX(x);
        bendPoint.setY(y);
        bendPoint.setTransition((Transition) modelManager.getPlanElement(mmq.getRelatedObjects().get(Types.TRANSITION)));
        return bendPoint;
    }

    @Override
    public void doCommand() {
        Transition transition = bendPoint.getTransition();
        this.uiElement.addBendpoint(index, bendPoint);

        this.modelManager.storePlanElement(mmq.getElementType(), bendPoint, false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, this.bendPoint.getTransition());
    }

    @Override
    public void undoCommand() {
        HashMap<String, Long> bendPointMap = new HashMap<>();
        bendPointMap.put(Types.BENDPOINT, bendPoint.getId());
        this.uiElement.removeBendpoint(this.bendPoint);
        this.modelManager.dropPlanElement(mmq.getElementType(), bendPoint, false);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, this.bendPoint.getTransition(), bendPointMap);
    }
}
