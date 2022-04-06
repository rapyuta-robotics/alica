package de.unikassel.vs.alica.planDesigner.command.copy;

import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.BendPoint;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiElement;
import de.unikassel.vs.alica.planDesigner.uiextensionmodel.UiExtension;

import java.util.HashMap;

public class CopyPlan extends Command {
    private Plan plan;
    private Plan copyPlan = new Plan();

    public CopyPlan(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.plan = (Plan) modelManager.getPlanElement(mmq.getElementId());
    }

    @Override
    public void doCommand() {
        if(plan.getMasterPlan()) {
            copyPlan.setMasterPlan(true);
        }
        copyPlan.setName(plan.getName() + "Copy");
        copyPlan.setComment(plan.getComment());
        copyPlan.setUtilityThreshold(plan.getUtilityThreshold());
        copyPlan.setMasterPlan(plan.getMasterPlan());
        copyPlan.setRelativeDirectory(modelManager.makeRelativeDirectory(plan.getRelativeDirectory(), copyPlan.getName()+ "." + Extensions.PLAN));
        //Set Variable
        for (Variable variable: plan.getVariables()) {
            Variable newVariable = new Variable();
            newVariable.setVariableType(variable.getVariableType());
            newVariable.setName(variable.getName());
            newVariable.setComment(variable.getComment());
            copyPlan.addVariable(newVariable);
        }
        //Set EntryPoints
        for (EntryPoint entryPoint: plan.getEntryPoints()) {
            EntryPoint newEntryPoint = new EntryPoint();
            if(entryPoint.getName().equals(String.valueOf(entryPoint.getId()))){
                newEntryPoint.setName(String.valueOf(newEntryPoint.getId()));
            } else {newEntryPoint.setName(entryPoint.getName()); }
            newEntryPoint.setMaxCardinality(entryPoint.getMaxCardinality());
            newEntryPoint.setMinCardinality(entryPoint.getMinCardinality());
            newEntryPoint.setPlan(copyPlan);
            newEntryPoint.setSuccessRequired(entryPoint.getSuccessRequired());
            newEntryPoint.setComment(entryPoint.getComment());
            newEntryPoint.setTask(entryPoint.getTask());
            copyPlan.addEntryPoint(newEntryPoint);
        }
        //Set Synchronisation
        for (Synchronisation synchronisation: plan.getSynchronisations()) {
            Synchronisation newSynchronisation = new Synchronisation();
            newSynchronisation.setPlan(copyPlan);
            newSynchronisation.setName(synchronisation.getName());
            newSynchronisation.setComment(synchronisation.getComment());
            newSynchronisation.setFailOnSyncTimeout(synchronisation.getFailOnSyncTimeout());
            newSynchronisation.setSyncTimeout(synchronisation.getSyncTimeout());
            newSynchronisation.setTalkTimeout(synchronisation.getTalkTimeout());
            copyPlan.addSynchronisation(newSynchronisation);
        }
        //Set State
        for (State state: plan.getStates()) {
            State newState;

            if(state instanceof TerminalState) {
                newState = new TerminalState(((TerminalState) state).isSuccess());
            } else {
                newState = new State();
                if(state.getEntryPoint() != null) {
                    for (int i = 0; i < plan.getEntryPoints().size(); i++) {
                        if(plan.getEntryPoints().get(i).getId() == state.getEntryPoint().getId()){
                            newState.setEntryPoint(copyPlan.getEntryPoints().get(i));
                            copyPlan.getEntryPoints().get(i).setState(newState);
                        }
                    }
                }
                for (ConfAbstractPlanWrapper confAbstractPlanWrapper : state.getConfAbstractPlanWrappers()) {
                    ConfAbstractPlanWrapper newConfAbstractPlanWrapper = new ConfAbstractPlanWrapper();
                    newConfAbstractPlanWrapper.setAbstractPlan(confAbstractPlanWrapper.getAbstractPlan());
                    newConfAbstractPlanWrapper.setConfiguration(confAbstractPlanWrapper.getConfiguration());
                    newState.addConfAbstractPlanWrapper(newConfAbstractPlanWrapper);
                }
            }

            newState.setName(state.getName());
            newState.setComment(state.getComment());
            newState.setParentPlan(copyPlan);
            copyPlan.addState(newState);
        }
        //Set Transition
        for (Transition transition: plan.getTransitions()) {
            Transition newTransition = new Transition();
            newTransition.setName(transition.getName());
            newTransition.setComment(transition.getComment());
            //Set PreCondition in Transition
            if(transition.getPreCondition() != null) {
                PreCondition newPreCondition = new PreCondition();
                newPreCondition.copyPreCondition(transition.getPreCondition(), plan, copyPlan);
                newTransition.setPreCondition(newPreCondition);
            }
            //Set connection between Synchronisation and Transition
            for (int i = 0; i < plan.getSynchronisations().size(); i++) {
                if(transition.getSynchronisation() != null &&
                        plan.getSynchronisations().get(i).getId() == transition.getSynchronisation().getId()){
                    newTransition.setSynchronisation(copyPlan.getSynchronisations().get(i));
                    copyPlan.getSynchronisations().get(i).addSyncedTransition(newTransition);
                }
            }
            //Set Transition InState connection
            for (int i = 0; i < plan.getStates().size(); i++) {
                if(transition.getInState().getId() == plan.getStates().get(i).getId()) {
                    newTransition.setInState(copyPlan.getStates().get(i));
                    copyPlan.getStates().get(i).addInTransition(newTransition);
                }
            }
            //Set Transition OutState connection
            for (int i = 0; i < plan.getStates().size(); i++) {
                if(transition.getOutState().getId() == plan.getStates().get(i).getId()) {
                    newTransition.setOutState(copyPlan.getStates().get(i));
                    copyPlan.getStates().get(i).addOutTransition(newTransition);
                }
            }
            copyPlan.addTransition(newTransition);
        }
        //Set PreCondition
        if(plan.getPreCondition() != null) {
            PreCondition newPreCondition = new PreCondition();
            newPreCondition.copyPreCondition(plan.getPreCondition(), plan, copyPlan);
            copyPlan.setPreCondition(newPreCondition);
        }
        //Set RuntimeCondition
        if(plan.getRuntimeCondition() != null) {
            RuntimeCondition newRuntimeCondition = new RuntimeCondition();
            newRuntimeCondition.copyRuntimeCondition(plan.getRuntimeCondition(), plan, copyPlan);
            copyPlan.setRuntimeCondition(newRuntimeCondition);
        }

        for (int i = 0; i <plan.getStates().size() ; i++) {
            //Set PostCondition in TerminalState
            if(plan.getStates().get(i) instanceof TerminalState &&
                    ((TerminalState) plan.getStates().get(i)).getPostCondition() != null){
                PostCondition newPostCondition = new PostCondition();
                newPostCondition.copyPostCondition(((TerminalState) plan.getStates().get(i)).getPostCondition(), plan, copyPlan);
                ((TerminalState) copyPlan.getStates().get(i)).setPostCondition(newPostCondition);
            }
            //Set VariableBindings in State
            for (VariableBinding variableBinding : plan.getStates().get(i).getVariableBindings()) {
                VariableBinding newVariableBinding = new VariableBinding();
                if(variableBinding.getName().equals(String.valueOf(variableBinding.getId()))){
                    newVariableBinding.setName(String.valueOf(newVariableBinding.getId()));
                } else {newVariableBinding.setName(variableBinding.getName()); }
                newVariableBinding.setComment(variableBinding.getComment());
                newVariableBinding.setSubPlan(variableBinding.getSubPlan());
                newVariableBinding.setSubVariable(variableBinding.getSubVariable());
                for (int j = 0; j < plan.getVariables().size(); j++) {
                    if(variableBinding.getVariable().getId() == plan.getVariables().get(j).getId()) {
                        newVariableBinding.setVariable(copyPlan.getVariables().get(j));
                    }
                }
                copyPlan.getStates().get(i).addVariableBinding(newVariableBinding);
            }
        }
        //Set UiElementMap for ...Copy.pmlex
        copyUiElements(plan, copyPlan);

        this.modelManager.storePlanElement(Types.PLAN, copyPlan,true);
        this.fireEvent(ModelEventType.ELEMENT_CREATED, copyPlan);
        this.modelManager.generateAutoGeneratedFilesForAbstractPlan(copyPlan);
    }
    @Override
    public void undoCommand() {
        modelManager.dropPlanElement(Types.PLAN, copyPlan, true);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, copyPlan);
    }

    private void copyUiElements(Plan plan, Plan copyPlan) {
        HashMap<Long, UiExtension> uiExtensionMap = modelManager.getUiExtensionMap();
        UiExtension newUiExtension = new UiExtension(copyPlan);
        UiExtension oldUiExtension = modelManager.getPlanUIExtensionPair(plan.getId());
        //Set State UIElement
        for (int i = 0; i < plan.getStates().size() ; i++) {
            UiElement newElement = newUiExtension.getUiElement(copyPlan.getStates().get(i).getId());
            UiElement oldElement = oldUiExtension.getUiElement(plan.getStates().get(i).getId());
            if(oldElement.getName().equals(String.valueOf(oldElement.getId()))){
                newElement.setName(String.valueOf(newElement.getId()));
            } else {newElement.setName(oldElement.getName()); }
            newElement.setComment(oldElement.getComment());
            newElement.setVisible(oldElement.isVisible());
            newElement.setX(oldElement.getX());
            newElement.setY(oldElement.getY());
            newUiExtension.add(copyPlan.getStates().get(i).getId(), newElement);
        }
        //Set Transition UIElement
        for (int i = 0; i < plan.getTransitions().size(); i++) {
            UiElement newElement = newUiExtension.getUiElement(copyPlan.getTransitions().get(i).getId());
            UiElement oldElement = oldUiExtension.getUiElement(plan.getTransitions().get(i).getId());
            if(oldElement.getName().equals(String.valueOf(oldElement.getId()))){
                newElement.setName(String.valueOf(newElement.getId()));
            } else {newElement.setName(oldElement.getName()); }
            newElement.setComment(oldElement.getComment());
            newElement.setVisible(oldElement.isVisible());
            newElement.setX(oldElement.getX());
            newElement.setY(oldElement.getY());
            for (BendPoint bendPoint: oldElement.getBendPoints()) {
                BendPoint newBendPoint = new BendPoint();
                if(bendPoint.getName().equals(String.valueOf(bendPoint.getId()))){
                    newBendPoint.setName(String.valueOf(newBendPoint.getId()));
                } else {newBendPoint.setName(bendPoint.getName()); }
                newBendPoint.setComment(bendPoint.getComment());
                newBendPoint.setX(bendPoint.getX());
                newBendPoint.setY(bendPoint.getY());

                if(plan.getTransitions().get(i).getId() == bendPoint.getTransition().getId()) {
                    newBendPoint.setTransition(copyPlan.getTransitions().get(i));
                }
                newElement.addBendpoint(newBendPoint);
            }
            newUiExtension.add(copyPlan.getTransitions().get(i).getId(), newElement);
        }
        //Set EntryPoint UIElement
        for (int i = 0; i < plan.getEntryPoints().size(); i++) {
            UiElement newElement = newUiExtension.getUiElement(copyPlan.getEntryPoints().get(i).getId());
            UiElement oldElement = oldUiExtension.getUiElement(plan.getEntryPoints().get(i).getId());
            if(oldElement.getName().equals(String.valueOf(oldElement.getId()))){
                newElement.setName(String.valueOf(newElement.getId()));
            } else {newElement.setName(oldElement.getName()); }
            newElement.setComment(oldElement.getComment());
            newElement.setVisible(oldElement.isVisible());
            newElement.setX(oldElement.getX());
            newElement.setY(oldElement.getY());
            newUiExtension.add(copyPlan.getEntryPoints().get(i).getId(), newElement);
        }
        uiExtensionMap.put(copyPlan.getId(), newUiExtension);
    }
}
