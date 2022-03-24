package de.unikassel.vs.alica.planDesigner.alicamodel;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.ListChangeListener;

public class ChangeListenerForDirtyFlag<T> implements ChangeListener<T>, ListChangeListener {

    SerializablePlanElement serializablePlanElement;

    public ChangeListenerForDirtyFlag(SerializablePlanElement serializablePlanElement) {
        this.serializablePlanElement = serializablePlanElement;
    }

    @Override
    public void changed(ObservableValue<? extends T> observable, T oldValue, T newValue) {
        this.serializablePlanElement.setDirty(true);
    }

    @Override
    public void onChanged(Change c) {
        this.serializablePlanElement.setDirty(true);
    }

    public void setDirty() {
        this.serializablePlanElement.setDirty(true);
    }
}
