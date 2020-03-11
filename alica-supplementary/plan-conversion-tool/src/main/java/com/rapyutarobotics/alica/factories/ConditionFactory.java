package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Condition;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class ConditionFactory extends Factory {
    public static void fillConditon(Element conditionNode, Condition condition) {
        Factory.setAttributes(conditionNode, condition);
        conversionTool.planElements.put(condition.getId(), condition);
        condition.setConditionString(conditionNode.getAttribute(Tags.CONDITIONSTRING));
        condition.setPluginName(conditionNode.getAttribute(Tags.PLUGINNAME));
        NodeList varNodes = conditionNode.getElementsByTagName(Tags.VARIABLES);
        for (int i = 0; i  < varNodes.getLength(); i++) {
            Element varNode = (Element) varNodes.item(i);
            Factory.conditionVarReferences.put(condition.getId(), Factory.getReferencedId(varNode.getTextContent()));
        }
        NodeList quantifierNodes = conditionNode.getElementsByTagName(Tags.QUANTIFIERS);
        for (int i = 0; i  < quantifierNodes.getLength(); i++) {
            Element quantifierNode = (Element) quantifierNodes.item(i);
            condition.addQuantifier(QuantifierFactory.create(quantifierNode));
        }
    }
}
