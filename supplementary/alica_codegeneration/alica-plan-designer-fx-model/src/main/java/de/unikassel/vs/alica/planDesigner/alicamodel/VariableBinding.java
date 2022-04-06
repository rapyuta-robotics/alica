package de.unikassel.vs.alica.planDesigner.alicamodel;

import javafx.beans.property.SimpleObjectProperty;

public class VariableBinding extends PlanElement{
    protected final SimpleObjectProperty<Variable> variable = new SimpleObjectProperty<>(this, "variable", null);
    protected final SimpleObjectProperty<AbstractPlan> subPlan = new SimpleObjectProperty<>(this, "subPlan", null);
    protected final SimpleObjectProperty<Variable> subVariable = new SimpleObjectProperty<>(this, "subVariable", null);

    public AbstractPlan getSubPlan() {
        return subPlan.get();
    }
    public void setSubPlan(AbstractPlan subPlan) {
        this.subPlan.set(subPlan);
    }
    public SimpleObjectProperty<AbstractPlan> subPlanProperty() {
        return subPlan;
    }

    public Variable getSubVariable() {
        return subVariable.get();
    }
    public void setSubVariable(Variable subVariable) {
        this.subVariable.set(subVariable);
    }
    public SimpleObjectProperty<Variable> subVariableProperty() {
        return subVariable;
    }

    public Variable getVariable() {
        return variable.get();
    }
    public void setVariable(Variable variable) {
        this.variable.set(variable);
    }
    public SimpleObjectProperty<Variable> variableProperty() {
        return variable;
    }

    public void registerDirtyFlag(ChangeListenerForDirtyFlag listener) {
        this.name.addListener(listener);
        this.comment.addListener(listener);
        this.subPlan.addListener(listener);
        this.subVariable.addListener(listener);
        this.variable.addListener(listener);
    }
}
