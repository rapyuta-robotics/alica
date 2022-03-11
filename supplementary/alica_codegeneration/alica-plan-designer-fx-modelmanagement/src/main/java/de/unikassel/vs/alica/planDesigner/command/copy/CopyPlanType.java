package de.unikassel.vs.alica.planDesigner.command.copy;

import de.unikassel.vs.alica.planDesigner.alicamodel.AnnotatedPlan;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanType;
import de.unikassel.vs.alica.planDesigner.alicamodel.Variable;
import de.unikassel.vs.alica.planDesigner.alicamodel.VariableBinding;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CopyPlanType  extends Command {
    private PlanType planType;
    private PlanType copyPlanType = new PlanType();
    public CopyPlanType(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.planType = (PlanType) modelManager.getPlanElement(mmq.getElementId());
    }
    @Override
    public void doCommand() {
        copyPlanType.setName(planType.getName() + "Copy");
        copyPlanType.setComment(planType.getComment());
        copyPlanType.setRelativeDirectory(modelManager.makeRelativeDirectory(planType.getRelativeDirectory(), copyPlanType.getName()+ "." + Extensions.PLANTYPE));
        //Set Variable
        for (Variable variable: planType.getVariables()) {
            Variable newVariable = new Variable();
            newVariable.setVariableType(variable.getVariableType());
            newVariable.setName(variable.getName());
            newVariable.setComment(variable.getComment());
            copyPlanType.addVariable(newVariable);
        }
        //Set AnnotatedPlans
        for (AnnotatedPlan annotatedPlan: planType.getAnnotatedPlans()) {
            AnnotatedPlan newAnnotatedPlan = new AnnotatedPlan();
            newAnnotatedPlan.setName(annotatedPlan.getName());
            newAnnotatedPlan.setComment(annotatedPlan.getComment());
            newAnnotatedPlan.setPlan(annotatedPlan.getPlan());
            newAnnotatedPlan.setActivated(annotatedPlan.isActivated());
            copyPlanType.addAnnotatedPlan(newAnnotatedPlan);
        }
        //Set VariableBinding
        for (VariableBinding variableBinding:planType.getVariableBindings()) {
            VariableBinding newVariableBinding = new VariableBinding();
            if(variableBinding.getName().equals(String.valueOf(variableBinding.getId()))){
                newVariableBinding.setName(String.valueOf(newVariableBinding.getId()));
            } else {newVariableBinding.setName(variableBinding.getName()); }
            newVariableBinding.setComment(variableBinding.getComment());
            newVariableBinding.setSubPlan(variableBinding.getSubPlan());
            newVariableBinding.setSubVariable(variableBinding.getSubVariable());
            for (int j = 0; j < planType.getVariables().size(); j++) {
                if(variableBinding.getVariable().getId() == planType.getVariables().get(j).getId()) {
                    newVariableBinding.setVariable(copyPlanType.getVariables().get(j));
                }
            }
            copyPlanType.addVariableBinding(newVariableBinding);
        }
        this.modelManager.storePlanElement(Types.PLANTYPE, copyPlanType,true);
        this.fireEvent(ModelEventType.ELEMENT_CREATED, copyPlanType);
    }
    @Override
    public void undoCommand() {
        modelManager.dropPlanElement(Types.PLANTYPE, copyPlanType, true);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, copyPlanType);
    }
}
