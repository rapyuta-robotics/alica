package de.unikassel.vs.alica.planDesigner.alicamodel;

import javafx.beans.property.SimpleObjectProperty;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

public class State extends PlanElement {

    protected final SimpleObjectProperty<EntryPoint> entryPoint = new SimpleObjectProperty<>();
    protected final SimpleObjectProperty<Plan> parentPlan = new SimpleObjectProperty<>();

    protected final ArrayList<ConfAbstractPlanWrapper> confAbstractPlanWrappers = new ArrayList<>();
    protected final ArrayList<VariableBinding> variableBindings = new ArrayList<>();
    protected final ArrayList<Transition> outTransitions = new ArrayList<>();
    protected final ArrayList<Transition> inTransitions = new ArrayList<>();

    private ChangeListenerForDirtyFlag changeListener;

    public State ()
    {
    }

    public State(long id) {
        this.id = id;
    }

    public EntryPoint getEntryPoint() {
        return entryPoint.get();
    }
    public void setEntryPoint(EntryPoint entryPoint) {
        this.entryPoint.set(entryPoint);
    }
    public SimpleObjectProperty<EntryPoint> entryPointProperty() { return entryPoint; }

    public Plan getParentPlan() {
        return parentPlan.get();
    }
    public void setParentPlan(Plan parentPlan) {
        this.parentPlan.set(parentPlan);
    }
    public SimpleObjectProperty<Plan> parentPlanProperty() { return parentPlan; }

    public List<ConfAbstractPlanWrapper> getConfAbstractPlanWrappers() {
        return Collections.unmodifiableList(confAbstractPlanWrappers);
    }
    public void addConfAbstractPlanWrapper(ConfAbstractPlanWrapper confAbstractPlanWrapper) {
        confAbstractPlanWrappers.add(confAbstractPlanWrapper);
        if (this.changeListener != null) {
            confAbstractPlanWrapper.registerDirtyFlag(this.changeListener);
            this.changeListener.setDirty();
        }
    }
    public void removeConfAbstractPlanWrapper(ConfAbstractPlanWrapper confAbstractPlanWrapper) {
        confAbstractPlanWrappers.remove(confAbstractPlanWrapper);
        // iterator in order to avoid concurrent modification exception
        Iterator<VariableBinding> iterator = variableBindings.iterator();
        while ((iterator).hasNext()) {
            VariableBinding param = iterator.next();
            if (param.getSubPlan().getId() == confAbstractPlanWrapper.getAbstractPlan().getId()) {
                iterator.remove();
            }
        }

        if (this.changeListener != null) {
            this.changeListener.setDirty();
        }
    }

    public void addVariableBinding(VariableBinding binding) {
        this.variableBindings.add(binding);
        if (this.changeListener != null) {
            binding.registerDirtyFlag(this.changeListener);
            this.changeListener.setDirty();
        }
    }
    public void removeVariableBinding(VariableBinding binding) {
        this.variableBindings.remove(binding);
        if (this.changeListener != null) {
            this.changeListener.setDirty();
        }
    }
    public List<VariableBinding> getVariableBindings() {
        return Collections.unmodifiableList(variableBindings);
    }

    public List<Transition> getOutTransitions() {
        return Collections.unmodifiableList(outTransitions);
    }
    public void addOutTransition(Transition transition) {
        this.outTransitions.add(transition);
    }
    public void removeOutTransition(Transition transition) {
        this.outTransitions.remove(transition);
    }

    public List<Transition> getInTransitions() {
        return Collections.unmodifiableList(inTransitions);
    }
    public void addInTransition(Transition transition) {
        this.inTransitions.add(transition);
    }
    public void removeInTransition(Transition transition) {
        this.inTransitions.remove(transition);
    }

    public void registerDirtyFlag(ChangeListenerForDirtyFlag listener) {
        this.changeListener = listener;
        this.name.addListener(listener);
        this.comment.addListener(listener);

        for (ConfAbstractPlanWrapper wrapper : confAbstractPlanWrappers) {
            wrapper.registerDirtyFlag(listener);
        }

        for (VariableBinding param : variableBindings) {
            param.registerDirtyFlag(listener);
        }
    }
}
