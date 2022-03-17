package de.unikassel.vs.alica.planDesigner.uiextensionmodel;

import de.unikassel.vs.alica.planDesigner.alicamodel.Transition;

public class BendPoint extends PositionedElement {
    private Transition transition;

    public Transition getTransition() {
        return transition;
    }

    public void setTransition(Transition transition) {
        this.transition = transition;
    }
}
