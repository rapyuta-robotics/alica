package de.unikassel.vs.alica.planDesigner.alicamodel;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.SimpleStringProperty;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Quantifier extends PlanElement {

    protected final SimpleStringProperty quantifierType = new SimpleStringProperty(this, "quantifierType", "");
    protected final SimpleObjectProperty<PlanElement> scope = new SimpleObjectProperty<>(this, "scope", null);
    protected final ArrayList<String> sorts = new ArrayList<>();

    private ChangeListenerForDirtyFlag changeListenerForDirtyFlag;

    public List<String> getSorts() {
        return Collections.unmodifiableList(sorts);
    }
    public void setSorts(List<String> sorts) {
        this.sorts.removeAll(this.getSorts());
        this.sorts.addAll(sorts);
        if(changeListenerForDirtyFlag != null) {
            changeListenerForDirtyFlag.setDirty();
        }
    }
    public void addSort(String sort) {
        sorts.add(sort);
        if (this.changeListenerForDirtyFlag != null) {
            changeListenerForDirtyFlag.setDirty();
        }
    }
    public void removeSort(String sort) {
        sorts.remove(sort);
        if (this.changeListenerForDirtyFlag != null) {
            changeListenerForDirtyFlag.setDirty();
        }
    }

    public PlanElement getScope() {
        return this.scope.getValue();
    }
    public void setScope(PlanElement scope) {
        this.scope.setValue(scope);
    }
    public ObjectProperty<PlanElement> scopeProperty() {
        return scope;
    }

    public String getQuantifierType() {
        return quantifierType.get();
    }
    public void setQuantifierType(String quantifierType) {
        this.quantifierType.set(quantifierType);
    }
    public SimpleStringProperty quantifierTypeProperty() {
        return quantifierType;
    }

    public void registerDirtyFlag(ChangeListenerForDirtyFlag listener) {
        if (listener == null) {
            return;
        }
        this.changeListenerForDirtyFlag = listener;
        this.name.addListener(listener);
        this.comment.addListener(listener);
        this.scope.addListener(listener);
        this.quantifierType.addListener(listener);
    }
}
