package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.AnnotatedPlan;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanType;
import de.unikassel.vs.alica.planDesigner.alicamodel.VariableBinding;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

import java.util.ArrayList;
import java.util.List;

public class DeleteAllAnnotatedPlans extends Command {

    private List<AnnotatedPlan> oldAnnotatedPlans;
    private PlanType planType;
    private List<VariableBinding> variableBindingList;

    public DeleteAllAnnotatedPlans(ModelManager manager, ModelModificationQuery mmq) {
        super(manager, mmq);
        this.planType = (PlanType) modelManager.getPlanElement(mmq.getParentId());
        this.oldAnnotatedPlans = new ArrayList<>(planType.getAnnotatedPlans());
        this.variableBindingList = new ArrayList<>(this.planType.getVariableBindings());
    }

    @Override
    public void doCommand() {
        // Remove all VariableBindings
        for (VariableBinding variableBinding: variableBindingList) {
            modelManager.dropPlanElement(Types.VARIABLEBINDING, variableBinding, false);
            this.planType.removeVariableBinding(variableBinding);
            this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, variableBinding);
        }

        // Remove all annotatedPlans
        for(AnnotatedPlan annotatedPlan : oldAnnotatedPlans) {
            planType.removeAnnotatedPlan(annotatedPlan);
            modelManager.dropPlanElement(Types.ANNOTATEDPLAN, annotatedPlan, false);
            this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, annotatedPlan);
        }
    }

    @Override
    public void undoCommand() {
        // Add all VariableBindings
        for (VariableBinding variableBinding: variableBindingList) {
            this.planType.addVariableBinding(variableBinding);
            modelManager.storePlanElement(Types.VARIABLEBINDING, variableBinding, false);
            this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, variableBinding);
        }

        for(AnnotatedPlan annotatedPlan : oldAnnotatedPlans) {
            planType.addAnnotatedPlan(annotatedPlan);
            modelManager.storePlanElement(Types.ANNOTATEDPLAN, annotatedPlan, false);
            this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, annotatedPlan);
        }
    }

}
