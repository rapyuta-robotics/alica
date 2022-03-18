package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.EntryPoint;
import de.unikassel.vs.alica.planDesigner.alicamodel.State;
import de.unikassel.vs.alica.planDesigner.command.UiPositionCommand;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiElement;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiExtension;

public class DeleteEntryPoint extends UiPositionCommand {

    protected State associatedState;
    protected final UiExtension parentOfElement;
    protected UiElement uiElement;
    protected EntryPoint entryPoint;

    public DeleteEntryPoint(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.entryPoint = (EntryPoint) modelManager.getPlanElement(mmq.getElementId());
        this.parentOfElement = modelManager.getPlanUIExtensionPair(mmq.getParentId());
        this.uiElement = parentOfElement.getUiElement(entryPoint.getId());
        this.associatedState = entryPoint.getState();
    }

    @Override
    public void doCommand() {
        parentOfElement.getPlan().removeEntryPoint(entryPoint);
        parentOfElement.remove(entryPoint.getId());
        if (associatedState != null) {
            associatedState.setEntryPoint(null);
        }
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, entryPoint);
    }

    @Override
    public void undoCommand() {
        parentOfElement.getPlan().addEntryPoint(entryPoint);
        parentOfElement.add(entryPoint.getId(), uiElement);
        if (associatedState != null) {
            associatedState.setEntryPoint(entryPoint);
        }
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, entryPoint);
    }
}
