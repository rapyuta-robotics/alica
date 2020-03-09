package com.rapyutarobotics.alica.factories;

import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;

public class Factory {

    static ModelManager modelManager;

    public static void setModelManager(ModelManager modelManager) {
        Factory.modelManager = modelManager;
    }
}
