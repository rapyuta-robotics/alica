package de.unikassel.vs.alica.planDesigner.alicamodel;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.SimpleStringProperty;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Condition extends PlanElement {

    protected final SimpleBooleanProperty enabled = new SimpleBooleanProperty(this, "enabled", false);
    protected final SimpleStringProperty conditionString = new SimpleStringProperty(this, "conditionString", null);
    protected final SimpleStringProperty pluginName = new SimpleStringProperty(this, "pluginName", null);
    protected final ArrayList<Variable> variables = new ArrayList<>();
    protected final ArrayList<Quantifier> quantifiers = new ArrayList<>();
    private ChangeListenerForDirtyFlag listenerForDirtyFlag;

    public boolean getEnabled () {
        return enabled.get();
    }
    public void setEnabled(boolean enabled) {
        this.enabled.set(enabled);
    }
    public SimpleBooleanProperty enabledProperty() {
        return enabled;
    }

    public String getConditionString() {
        return conditionString.get();
    }
    public void setConditionString(String conditionString) {
        this.conditionString.set(conditionString);
    }
    public SimpleStringProperty conditionStringProperty() {
        return conditionString;
    }

    public String getPluginName() {
        return pluginName.get();
    }
    public void setPluginName(String pluginName) {
        this.pluginName.set(pluginName);
    }
    public SimpleStringProperty pluginNameProperty() {
        return pluginName;
    }

    public List<Variable> getVariables() {
        return Collections.unmodifiableList(variables);
    }
    public void addVariable(Variable variable){
        variables.add(variable);
        if (listenerForDirtyFlag != null) {
            listenerForDirtyFlag.setDirty();
        }
    }
    public void removeVariable(Variable variable){
        variables.remove(variable);
        if (listenerForDirtyFlag != null) {
            listenerForDirtyFlag.setDirty();
        }
    }

    public List<Quantifier> getQuantifiers() {
        return Collections.unmodifiableList(quantifiers);
    }
    public void addQuantifier(Quantifier quantifier) {
        quantifiers.add(quantifier);
        quantifier.registerDirtyFlag(listenerForDirtyFlag);
        if (listenerForDirtyFlag != null) {
            listenerForDirtyFlag.setDirty();
        }
    }
    public void removeQuantifier(Quantifier quantifier) {
        quantifiers.remove(quantifier);
        if (listenerForDirtyFlag != null) {
            listenerForDirtyFlag.setDirty();
        }
    }

    public void registerDirtyFlag(ChangeListenerForDirtyFlag listener) {
        if (listener == null) {
            return;
        }
        this.listenerForDirtyFlag = listener;

        this.name.addListener(listener);
        this.comment.addListener(listener);
        this.enabled.addListener(listener);
        this.conditionString.addListener(listener);
        this.pluginName.addListener(listener);

        for (Quantifier quantifier : quantifiers) {
            quantifier.registerDirtyFlag(listener);
        }
        // Variables are owned by an AbstractPlan, so listeners are registered there and exist already
    }

    public boolean hasConstraint() {
        return !variables.isEmpty() || !quantifiers.isEmpty();
    }
}