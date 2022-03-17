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

public class DeleteAnnotatedPlan extends Command {

    protected PlanType planType;
    protected AnnotatedPlan annotatedPlan;
    private List<VariableBinding> variableBindingList;

    public DeleteAnnotatedPlan(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        planType = (PlanType) modelManager.getPlanElement(mmq.getParentId());
        variableBindingList = new ArrayList<>(this.planType.getVariableBindings());
        annotatedPlan = (AnnotatedPlan) modelManager.getPlanElement(mmq.getElementId());
    }

    @Override
    public void doCommand() {
        for (VariableBinding variableBinding: variableBindingList) {
            if(variableBinding.getSubPlan().getId() == this.annotatedPlan.getPlan().getId()) {
                this.planType.removeVariableBinding(variableBinding);
                this.modelManager.dropPlanElement(Types.VARIABLEBINDING, variableBinding, false);
                this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, variableBinding);
            }
        }

        this.planType.removeAnnotatedPlan(this.annotatedPlan);
        this.modelManager.dropPlanElement(Types.ANNOTATEDPLAN, this.annotatedPlan, false);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, this.annotatedPlan);
    }

    @Override
    public void undoCommand() {
        for (VariableBinding variableBinding : variableBindingList) {
            if (variableBinding.getSubPlan().getId() == this.annotatedPlan.getPlan().getId()) {
                this.planType.addVariableBinding(variableBinding);
                this.modelManager.storePlanElement(Types.VARIABLEBINDING, variableBinding, false);
                this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, variableBinding);
            }
        }

        this.planType.addAnnotatedPlan(this.annotatedPlan);
        this.modelManager.storePlanElement(Types.ANNOTATEDPLAN, this.annotatedPlan, false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, this.annotatedPlan);
    }
}
