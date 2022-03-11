package de.unikassel.vs.alica.planDesigner.alicamodel;

import javafx.beans.property.SimpleObjectProperty;

public class Transition extends  PlanElement {
    protected final SimpleObjectProperty<State> inState = new SimpleObjectProperty<>(this, "inState", null);
    protected final SimpleObjectProperty<State> outState = new SimpleObjectProperty<>(this, "outState", null);
    protected final SimpleObjectProperty<PreCondition> preCondition = new SimpleObjectProperty<>(this, "preCondition", null);
    protected final SimpleObjectProperty<Synchronisation> synchronisation = new SimpleObjectProperty<>();

    private ChangeListenerForDirtyFlag changeListener;

    public Transition(long id) {
        this.id = id;
    }

    public Transition() {
    }

    public State getInState() {
        return inState.get();
    }
    public void setInState(State inState) {
        this.inState.set(inState);
    }
    public SimpleObjectProperty<State> inStateProperty() {
        return inState;
    }

    public State getOutState() {
        return outState.get();
    }
    public void setOutState(State outState) {
        this.outState.set(outState);
    }
    public SimpleObjectProperty<State> outStateProperty() {
        return outState;
    }

    public Synchronisation getSynchronisation() {
        return synchronisation.get();
    }
    public void setSynchronisation(Synchronisation synchronisation) {
        this.synchronisation.set(synchronisation);
    }
    public SimpleObjectProperty<Synchronisation> synchronisationProperty() {
        return synchronisation;
    }

    public PreCondition getPreCondition() {
        return preCondition.get();
    }
    public void setPreCondition(PreCondition preCondition) {
        this.preCondition.set(preCondition);
        if(preCondition != null) {
            preCondition.registerDirtyFlag(this.changeListener);
        }
    }
    public SimpleObjectProperty<PreCondition> preConditionProperty() {
        return preCondition;
    }

    public void registerDirtyFlag(ChangeListenerForDirtyFlag listener) {
        if (listener == null) {
            return;
        }
        this.changeListener = listener;
        this.name.addListener(listener);
        this.comment.addListener(listener);
        if (getPreCondition() != null) {
            this.getPreCondition().registerDirtyFlag(listener);
        }
    }
}
