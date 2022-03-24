package de.unikassel.vs.alica.planDesigner.alicamodel;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleLongProperty;
import javafx.beans.property.SimpleObjectProperty;

public class Behaviour extends AbstractPlan {
    protected final SimpleIntegerProperty frequency = new SimpleIntegerProperty(this, "frequency", 0);
    protected final SimpleLongProperty deferring = new SimpleLongProperty(this, "deferring", 0);
    protected final SimpleBooleanProperty eventDriven = new SimpleBooleanProperty(this, "eventDriven", false);

    protected SimpleObjectProperty<PreCondition> preCondition = new SimpleObjectProperty<>();
    protected SimpleObjectProperty<RuntimeCondition> runtimeCondition = new SimpleObjectProperty<>();
    protected SimpleObjectProperty<PostCondition> postCondition = new SimpleObjectProperty<>();

    public PreCondition getPreCondition() {
        return preCondition.get();
    }
    public void setPreCondition(PreCondition preCondition) {
        this.preCondition.set(preCondition);
        if(preCondition != null) {
            preCondition.registerDirtyFlag(this.changeListenerForDirtyFlag);
        }
    }
    public SimpleObjectProperty<PreCondition> preConditionProperty() {
        return preCondition;
    }

    public RuntimeCondition getRuntimeCondition() {
        return runtimeCondition.get();
    }
    public void setRuntimeCondition(RuntimeCondition runtimeCondition) {
        this.runtimeCondition.set(runtimeCondition);
        if(runtimeCondition != null) {
            runtimeCondition.registerDirtyFlag(this.changeListenerForDirtyFlag);
        }
    }
    public SimpleObjectProperty<RuntimeCondition> runtimeConditionProperty() {
        return runtimeCondition;
    }

    public PostCondition getPostCondition() {
        return postCondition.get();
    }
    public void setPostCondition(PostCondition postCondition) {
        this.postCondition.set(postCondition);
        if(postCondition != null) {
            postCondition.registerDirtyFlag(this.changeListenerForDirtyFlag);
        }
    }
    public SimpleObjectProperty<PostCondition> postConditionProperty() {
        return postCondition;
    }

    public int getFrequency() {
        return frequency.get();
    }
    public void setFrequency(int frequency) {
        this.frequency.set(frequency);
    }
    public SimpleIntegerProperty frequencyProperty() {
        return frequency;
    }

    public long getDeferring() {
        return deferring.get();
    }
    public void setDeferring(long deferring) {
        this.deferring.set(deferring);
    }
    public SimpleLongProperty deferringProperty() {
        return this.deferring;
    }

    public boolean isEventDriven() {
        return eventDriven.get();
    }
    public void setEventDriven(boolean eventDriven) {
        this.eventDriven.set(eventDriven);
    }
    public SimpleBooleanProperty eventDrivenProperty() {
        return this.eventDriven;
    }

    @Override
    public void registerDirtyFlag() {
        super.registerDirtyFlag();
        this.frequency.addListener(this.changeListenerForDirtyFlag);
        this.deferring.addListener(this.changeListenerForDirtyFlag);
        this.preCondition.addListener(this.changeListenerForDirtyFlag);
        if (this.preCondition.get() != null) {
            this.preCondition.get().registerDirtyFlag(this.changeListenerForDirtyFlag);
        }
        this.runtimeCondition.addListener(this.changeListenerForDirtyFlag);
        if (this.runtimeCondition.get() != null) {
            this.runtimeCondition.get().registerDirtyFlag(this.changeListenerForDirtyFlag);
        }
        this.postCondition.addListener(this.changeListenerForDirtyFlag);
        if (this.postCondition.get() != null) {
            this.postCondition.get().registerDirtyFlag(this.changeListenerForDirtyFlag);
        }
    }

    @Override
    public void removeVariable(Variable variable) {
        super.removeVariable(variable);

        // Remove the Variable from all child-elements
        if(getPreCondition() != null) {
            getPreCondition().removeVariable(variable);
        }
        if(getRuntimeCondition() != null) {
            getRuntimeCondition().removeVariable(variable);
        }
        if(getPostCondition() != null) {
            getPostCondition().removeVariable(variable);
        }
    }
}
