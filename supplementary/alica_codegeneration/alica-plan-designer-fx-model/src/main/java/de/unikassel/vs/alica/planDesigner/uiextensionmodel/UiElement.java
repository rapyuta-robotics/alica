package de.unikassel.vs.alica.planDesigner.uiextensionmodel;

import javafx.beans.property.*;

import java.util.ArrayList;
import java.util.LinkedList;

public class UiElement extends PositionedElement {

    protected final SimpleBooleanProperty visible = new SimpleBooleanProperty();

    protected LinkedList<BendPoint> bendPoints;

    public UiElement() {
        bendPoints = new LinkedList<>();
    }

    public LinkedList<BendPoint> getBendPoints() {return this.bendPoints;}

    public boolean isVisible() {return this.visible.get();}

    public void setVisible(boolean visible) {this.visible.set(visible);}

    public SimpleBooleanProperty visibleProperty() {
        return visible;
    }

    public void addBendpoint(BendPoint bendPoint) {
        bendPoints.add(bendPoint);
    }

    public void addBendpoint(int index, BendPoint bendPoint) {
        bendPoints.add(index, bendPoint);
    }

    public void removeBendpoint(BendPoint bendPoint) {
        bendPoints.remove(bendPoint);
    }
}
