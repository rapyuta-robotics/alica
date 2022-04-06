package de.unikassel.vs.alica.planDesigner.alicamodel;


import javafx.beans.property.SimpleDoubleProperty;

public class Characteristic extends PlanElement {

    protected final SimpleDoubleProperty weight = new SimpleDoubleProperty();
//    protected Capability capability;
    protected String value;
    protected Role role;

    public double getWeight() {
        return this.weight.get();
    }
    public void setWeight(double weight) {
        this.weight.set(weight);
    }
    public SimpleDoubleProperty weightProperty() {
        return weight;
    }

//    public Capability getCapability() {
//        return this.capability;
//    }
//    public void setCapability(Capability capability) {
//        this.capability = capability;
//    }

    public String getValue() {
        return this.value;
    }
    public void setValue(String value) {
        this.value = value;
    }

    public Role getRole() {
        return role;
    }
    public void setRole(Role role) {
        this.role = role;
    }

    public void addListener(ChangeListenerForDirtyFlag listener) {
        weight.addListener(listener);
    }
}
