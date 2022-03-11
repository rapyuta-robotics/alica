package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.Synchronisation;
import de.unikassel.vs.alica.planDesigner.alicamodel.Transition;
import de.unikassel.vs.alica.planDesigner.command.UiPositionCommand;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiExtension;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiElement;

import java.util.List;

public class DeleteSynchronisationFromPlan extends UiPositionCommand {

    protected UiExtension parentOfElement;
    protected UiElement uiElement;
    protected Synchronisation synchronisation;
    private List<Transition> transitionList;

    public DeleteSynchronisationFromPlan(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.synchronisation = (Synchronisation) this.modelManager.getPlanElement(mmq.getElementId());
        this.parentOfElement = this.modelManager.getPlanUIExtensionPair(mmq.getParentId());
    }

    @Override
    public void doCommand() {
        parentOfElement.getPlan().removeSynchronisation(synchronisation);
        uiElement = parentOfElement.getUiElement(synchronisation.getId());
        parentOfElement.remove(synchronisation.getId());

        transitionList = synchronisation.getSyncedTransitions();
        for (Transition transition: transitionList) {
            transition.setSynchronisation(null);
        }
        this.modelManager.dropPlanElement(Types.SYNCHRONISATION, synchronisation, true);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, synchronisation);
    }

    @Override
    public void undoCommand() {
        parentOfElement.getPlan().addSynchronisation(synchronisation);
        parentOfElement.add(synchronisation.getId(), this.uiElement);

        for (Transition transition: transitionList) {
            transition.setSynchronisation(synchronisation);
        }
        modelManager.storePlanElement(Types.SYNCHRONISATION, synchronisation,false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, synchronisation);
    }
}
