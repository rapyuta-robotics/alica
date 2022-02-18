package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Variable;
import org.w3c.dom.Element;

public class VariableFactory extends Factory {

    public static Variable create(Element variableNode, ConversionProcess cp) {
        Variable variable = new Variable();
        Factory.setAttributes(variableNode, variable);
        cp.addElement(variable);
        variable.setVariableType(variableNode.getAttribute(Tags.VARIABLETYPE));
        return variable;
    }
}
