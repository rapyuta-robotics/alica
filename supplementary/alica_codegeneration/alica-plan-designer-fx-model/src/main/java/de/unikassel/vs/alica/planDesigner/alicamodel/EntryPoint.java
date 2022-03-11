package de.unikassel.vs.alica.planDesigner.alicamodel;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleObjectProperty;

public class EntryPoint extends PlanElement {

    protected final SimpleBooleanProperty successRequired = new SimpleBooleanProperty(this, "successRequired", false);
    protected final SimpleIntegerProperty minCardinality = new SimpleIntegerProperty(this, "minCardinality", 0);
    protected final SimpleIntegerProperty maxCardinality = new SimpleIntegerProperty(this, "maxCardinality", 2147483647);
    protected final SimpleObjectProperty<Task> task = new SimpleObjectProperty<>(this, "task", null);
    protected final SimpleObjectProperty<State> state = new SimpleObjectProperty<>(this, "state", null);
    protected final SimpleObjectProperty<Plan> plan = new SimpleObjectProperty<>(this, "plan", null);


    public boolean getSuccessRequired() {
        return successRequired.get();
    }
    public void setSuccessRequired(boolean successRequired) {
        this.successRequired.set(successRequired);
    }
    public SimpleBooleanProperty successRequiredProperty() {
        return successRequired;
    }

    public int getMinCardinality() {
        return minCardinality.get();
    }
    public void setMinCardinality(int minCardinality) {
        this.minCardinality.set(minCardinality);
    }
    public SimpleIntegerProperty minCardinalityProperty() {
        return minCardinality;
    }

    public int getMaxCardinality() {
        return maxCardinality.get();
    }
    public void setMaxCardinality(int maxCardinality) {
        this.maxCardinality.set(maxCardinality);
    }
    public SimpleIntegerProperty maxCardinalityProperty() {
        return maxCardinality;
    }

    public Task getTask() {
        return task.get();
    }
    public void setTask(Task task) {
        this.task.set(task);
    }
    public SimpleObjectProperty<Task> taskProperty() {
        return task;
    }

    public State getState() {
        return state.get();
    }
    public void setState(State state) {
        this.state.set(state);
    }
    public SimpleObjectProperty<State> stateProperty() {
        return state;
    }

    public Plan getPlan() {
        return plan.get();
    }
    public void setPlan(Plan plan) {
        this.plan.set(plan);
    }
    public SimpleObjectProperty<Plan> planProperty() {
        return plan;
    }

    public void registerDirtyFlag(ChangeListenerForDirtyFlag listener)
    {
        this.successRequired.addListener(listener);
        this.minCardinality.addListener(listener);
        this.maxCardinality.addListener(listener);
        this.task.addListener(listener);
        this.state.addListener(listener);
        this.plan.addListener(listener);
    }
}
