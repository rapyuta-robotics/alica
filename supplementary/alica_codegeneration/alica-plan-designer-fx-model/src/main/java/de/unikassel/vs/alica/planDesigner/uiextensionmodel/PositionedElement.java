package de.unikassel.vs.alica.planDesigner.uiextensionmodel;

import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import javafx.beans.property.SimpleIntegerProperty;

public class PositionedElement extends PlanElement {

    protected final SimpleIntegerProperty x = new SimpleIntegerProperty();
    protected final SimpleIntegerProperty y = new SimpleIntegerProperty();

    public int getX() {return this.x.get();}

    public void setX(int x) {this.x.set(x);}

    public SimpleIntegerProperty xProperty() {
        return x;
    }

    public int getY() {
        return this.y.get();
    }

    public void setY(int y) {
        this.y.set(y);
    }

    public SimpleIntegerProperty yProperty() {
        return y;
    }
}
