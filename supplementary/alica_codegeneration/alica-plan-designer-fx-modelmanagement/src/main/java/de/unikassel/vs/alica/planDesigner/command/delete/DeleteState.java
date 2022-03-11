package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.command.UiPositionCommand;
import de.unikassel.vs.alica.planDesigner.command.remove.RemoveAbstractPlanFromState;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiExtension;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiElement;

import java.util.ArrayList;
import java.util.List;

public class DeleteState extends UiPositionCommand {

    protected State state;
    protected Plan plan;
    private UiElement uiElement;
    private final UiExtension parentOfElement;
    private EntryPoint entryPoint;
    private List<DeleteTransition> deleteTransitionInPlansList = new ArrayList<>();
    private List<DeleteVariableBinding> deleteVariableBindingList = new ArrayList<>();
    private List<RemoveAbstractPlanFromState> removeAbstractPlanFromStateList = new ArrayList<>();

    public DeleteState(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.state = (State) modelManager.getPlanElement(mmq.getElementId());
        this.plan = (Plan) modelManager.getPlanElement(mmq.getParentId());
        this.parentOfElement = this.modelManager.getPlanUIExtensionPair(mmq.getParentId());
    }

    @Override
    public void doCommand() {
        if(state.getVariableBindings().size() != 0){
            deleteVariableBindings(state.getVariableBindings());
        }
        if(state.getInTransitions().size() !=0 || state.getOutTransitions().size() != 0){
            if(state.getInTransitions().size() != 0){
                deleteTransitions(state.getInTransitions());
            }
            if(state.getOutTransitions().size() !=0) {
                deleteTransitions(state.getOutTransitions());
            }
            for (DeleteTransition d: deleteTransitionInPlansList) { d.doCommand(); }
        }

        if(state.getConfAbstractPlanWrappers().size() != 0) {
            deleteAbstractPlans(state.getConfAbstractPlanWrappers());
        }

        if(state.getEntryPoint() != null) {
            entryPoint = state.getEntryPoint();
            state.setEntryPoint(null);
            entryPoint.setState(null);
        }

        mmq.setElementId(state.getId());
        mmq.setParentId(plan.getId());

        uiElement = parentOfElement.getUiElement(state.getId());
        parentOfElement.remove(state.getId());
        parentOfElement.getPlan().removeState(state);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, state);
    }

    @Override
    public void undoCommand() {
        parentOfElement.getPlan().addState(state);
        parentOfElement.add(state.getId(), uiElement);

        if(entryPoint != null){
            state.setEntryPoint(entryPoint);
            entryPoint.setState(state);
        }
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, state);

        if(deleteTransitionInPlansList != null) {
            mmq.setElementType(Types.TRANSITION);
            mmq.setParentId(plan.getId());
            for (DeleteTransition d: deleteTransitionInPlansList) { d.undoCommand(); }
        }
        if(deleteVariableBindingList != null){
            mmq.setElementType(Types.VARIABLEBINDING);
            mmq.setParentId(state.getId());
            for (DeleteVariableBinding d: deleteVariableBindingList) { d.undoCommand(); }
        }
        if(removeAbstractPlanFromStateList != null) {
            mmq.setElementType(Types.PLAN);
            mmq.setParentId(state.getId());
            for (RemoveAbstractPlanFromState d: removeAbstractPlanFromStateList) { d.undoCommand(); }
        }
    }

    private void deleteVariableBindings(List<VariableBinding> variableBindings) {
        for (VariableBinding variableBinding: variableBindings) {
            ModelModificationQuery variableBindingMMQ = mmq;
            variableBindingMMQ.setParentId(state.getId());
            variableBindingMMQ.setElementId(variableBinding.getId());
            DeleteVariableBinding deleteVariableBinding = new DeleteVariableBinding(modelManager, variableBindingMMQ);
            deleteVariableBindingList.add(deleteVariableBinding);
        }
        for (DeleteVariableBinding d: deleteVariableBindingList) { d.doCommand(); }
    }

    private void deleteAbstractPlans(List<ConfAbstractPlanWrapper> wrappers) {
        for (ConfAbstractPlanWrapper wrapper: wrappers) {
            ModelModificationQuery abstractPlanMMQ = mmq;
            abstractPlanMMQ.setParentId(state.getId());
            abstractPlanMMQ.setElementId(wrapper.getId());
            RemoveAbstractPlanFromState removeAbstractPlanFromState = new RemoveAbstractPlanFromState(modelManager, abstractPlanMMQ);
            removeAbstractPlanFromStateList.add(removeAbstractPlanFromState);
        }
        for (RemoveAbstractPlanFromState d: removeAbstractPlanFromStateList) { d.doCommand(); }
    }

    private void deleteTransitions(List<Transition> transitions) {
        for (Transition transition: transitions) {
            ModelModificationQuery transitionMMQ = mmq;
            transitionMMQ.setParentId(plan.getId());
            transitionMMQ.setElementId(transition.getId());
            DeleteTransition deleteTransition = new DeleteTransition(modelManager, transitionMMQ);
            deleteTransitionInPlansList.add(deleteTransition);
        }
    }
}
