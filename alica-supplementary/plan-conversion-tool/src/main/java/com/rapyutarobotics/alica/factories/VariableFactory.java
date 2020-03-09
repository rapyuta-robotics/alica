package com.rapyutarobotics.alica.factories;

import de.unikassel.vs.alica.planDesigner.alicamodel.Variable;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import org.w3c.dom.Element;

public class VariableFactory extends Factory {

    public static Variable create(Element variableNode) {
        Variable variable = new Variable();
        Factory.setAttributes(variableNode, variable);
        Factory.storeElement(variable, Types.VARIABLE);
        variable.setVariableType(variableNode.getAttribute(VARIABLETYPE));
        return variable;
    }
}
