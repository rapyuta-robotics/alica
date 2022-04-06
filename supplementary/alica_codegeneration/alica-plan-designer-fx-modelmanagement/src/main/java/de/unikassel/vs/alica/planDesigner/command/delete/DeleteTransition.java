package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.PreCondition;
import de.unikassel.vs.alica.planDesigner.alicamodel.State;
import de.unikassel.vs.alica.planDesigner.alicamodel.Synchronisation;
import de.unikassel.vs.alica.planDesigner.alicamodel.Transition;
import de.unikassel.vs.alica.planDesigner.command.UiPositionCommand;
import de.unikassel.vs.alica.planDesigner.events.ModelEvent;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiElement;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiExtension;

public class DeleteTransition extends UiPositionCommand {

    private final UiExtension parentOfElement;
    private UiElement uiElement;
    private State inState;
    private State outState;
    private Transition transition;
    private Synchronisation synchronisation;
    private PreCondition preCondition;

    public DeleteTransition(ModelManager manager, ModelModificationQuery mmq) {
        super(manager, mmq);
        this.parentOfElement = this.modelManager.getPlanUIExtensionPair(mmq.getParentId());
        this.transition = (Transition) this.modelManager.getPlanElement(mmq.getElementId());
    }

    private void saveForLaterRetrieval() {
        uiElement = parentOfElement.getUiElement(transition.getId());
        outState = transition.getOutState();
        inState = transition.getInState();
    }

    @Override
    public void doCommand() {
        saveForLaterRetrieval();
        parentOfElement.remove(transition.getId());
        parentOfElement.getPlan().removeTransition(transition);

        inState.removeOutTransition(transition);
        outState.removeInTransition(transition);

        preCondition = transition.getPreCondition();
        transition.setPreCondition(null);
        transition.setInState(null);
        transition.setOutState(null);

        if(transition.getSynchronisation() != null) {
            synchronisation = transition.getSynchronisation();
            synchronisation.removeSyncedTransition(transition);
            this.transition.setSynchronisation(null);
            ModelEvent event = new ModelEvent(ModelEventType.ELEMENT_DISCONNECTED, transition, Types.SYNCTRANSITION);
            event.setParentId(synchronisation.getId());
            this.modelManager.fireEvent(event);
        }

        this.modelManager.dropPlanElement(Types.TRANSITION, transition, true);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, transition);
    }

    @Override
    public void undoCommand() {
        parentOfElement.getPlan().addTransition(transition);
        parentOfElement.add(transition.getId(), uiElement);

        transition.setPreCondition(preCondition);
        transition.setInState(inState);
        transition.setOutState(outState);

        inState.addOutTransition(transition);
        outState.addInTransition(transition);

        if(synchronisation != null) {
            transition.setSynchronisation(synchronisation);
            synchronisation.addSyncedTransition(transition);
        }

        modelManager.storePlanElement(Types.TRANSITION, transition,false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, transition);
    }
}
