package de.unikassel.vs.alica.planDesigner.alicamodel;

public class RuntimeCondition extends Condition {

    public RuntimeCondition copyRuntimeCondition(RuntimeCondition oldRuntimeCondition, AbstractPlan oldAbstractPlan, AbstractPlan copyAbstractPlan) {
        //If ID = Name
        if(oldRuntimeCondition.getName().equals(String.valueOf(oldRuntimeCondition.getId()))){
            this.setName(String.valueOf(this.getId()));
        } else { this.setName(oldRuntimeCondition.getName()); }
        this.setComment(oldRuntimeCondition.getComment());
        this.setConditionString(oldRuntimeCondition.getConditionString());
        this.setEnabled(oldRuntimeCondition.getEnabled());
        this.setPluginName(oldRuntimeCondition.getPluginName());

        for (Variable oldRuntimeConditionVariable: oldRuntimeCondition.getVariables()) {
            for (int i = 0; i < oldAbstractPlan.getVariables().size(); i++) {
                if(oldAbstractPlan.getVariables().get(i).getId() == oldRuntimeConditionVariable.getId()) {
                    this.variables.add(copyAbstractPlan.getVariables().get(i));
                }
            }
        }

        //Set Quantifier only for Plan or MasterPlan
        if(oldAbstractPlan instanceof Plan) {
            for (Quantifier quantifier : oldRuntimeCondition.getQuantifiers()) {
                Quantifier newQuantifier = new Quantifier();
                Plan oldPlan = (Plan) oldAbstractPlan;
                Plan copyPlan = (Plan) copyAbstractPlan;
                newQuantifier.setName(quantifier.getName());
                newQuantifier.setComment(quantifier.getComment());
                newQuantifier.setQuantifierType(quantifier.getQuantifierType());
                if (quantifier.getScope() != null) {
                    if (quantifier.getScope().getId() == oldAbstractPlan.getId()) {
                        newQuantifier.setScope(copyAbstractPlan);
                    }
                    if (quantifier.getScope() instanceof State) {
                        for (int i = 0; i < oldPlan.getStates().size(); i++) {
                            if(oldPlan.getStates().get(i).getId() == quantifier.getScope().getId()){
                                newQuantifier.setScope(copyPlan.getStates().get(i));
                            }
                        }
                    }
                    if (quantifier.getScope() instanceof EntryPoint) {
                        for (int i = 0; i < oldPlan.getEntryPoints().size() ; i++) {
                            if(oldPlan.getEntryPoints().get(i).getId() == quantifier.getScope().getId()){
                                newQuantifier.setScope(copyPlan.getEntryPoints().get(i));
                            }
                        }
                    }
                }
                newQuantifier.sorts.addAll(quantifier.getSorts());
                this.quantifiers.add(newQuantifier);
            }
        }
        return this;
    }
}
