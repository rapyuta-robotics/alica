package de.unikassel.vs.alica.planDesigner.command.change;

import de.unikassel.vs.alica.planDesigner.command.UiPositionCommand;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.BendPoint;

public class ChangePositionForBendpoint extends UiPositionCommand {

    protected BendPoint bendPoint;
    protected int oldX;
    protected int oldY;

    public ChangePositionForBendpoint(ModelManager manager, ModelModificationQuery mmq) {
        super(manager, mmq);
        //TODO: depending on how the mmq member variables are set, this command probably won't work
        this.bendPoint = (BendPoint) this.modelManager.getPlanElement(mmq.getElementId());
        this.oldX = bendPoint.getX();
        this.oldY = bendPoint.getY();
    }

    @Override
    public void doCommand() {
        bendPoint.setX(x);
        bendPoint.setY(y);
        this.fireEvent(ModelEventType.ELEMENT_ATTRIBUTE_CHANGED, this.bendPoint);
    }

    @Override
    public void undoCommand() {
        bendPoint.setX(oldX);
        bendPoint.setY(oldY);
        this.fireEvent(ModelEventType.ELEMENT_ATTRIBUTE_CHANGED, this.bendPoint);
    }
}
