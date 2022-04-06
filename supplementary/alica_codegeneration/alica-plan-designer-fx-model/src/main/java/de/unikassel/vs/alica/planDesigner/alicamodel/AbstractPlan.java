package de.unikassel.vs.alica.planDesigner.alicamodel;


import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AbstractPlan extends SerializablePlanElement {

    protected final ArrayList<Variable> variables = new ArrayList<>();

    public AbstractPlan () {
        super();
    }
    public AbstractPlan (long id) {
        this.id = id;
    }

    public void addVariable(Variable variable) {
        variable.registerDirtyFlag(this.changeListenerForDirtyFlag);
        variables.add(variable);
        this.changeListenerForDirtyFlag.setDirty();
    }
    public void removeVariable(Variable variable) {
        variables.remove(variable);
        this.changeListenerForDirtyFlag.setDirty();
    }
    public List<Variable> getVariables() {
        return Collections.unmodifiableList(variables);
    }

    public void registerDirtyFlag() {
        super.registerDirtyFlag();
        for (Variable var : variables) {
            var.registerDirtyFlag(this.changeListenerForDirtyFlag);
        }
    }
}
