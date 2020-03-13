package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.AbstractPlan;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class AbstractPlanFactory extends Factory {

    public static void setVariables(Element node, AbstractPlan abstractPlan, ConversionProcess cp)
    {
        NodeList listOfVariableNodes = node.getElementsByTagName(Tags.VARIABLES);
        for (int i = 0; i < listOfVariableNodes.getLength(); i++) {
            Element variableNode = (Element) listOfVariableNodes.item(i);
            if (variableNode.getParentNode().getNodeName().equals(Tags.PLANTAG)) {
                // filter for those <vars..> nodes that are direct children of <alica:plan..> nodes
                abstractPlan.addVariable(VariableFactory.create(variableNode, cp));
            }
        }
    }
}
