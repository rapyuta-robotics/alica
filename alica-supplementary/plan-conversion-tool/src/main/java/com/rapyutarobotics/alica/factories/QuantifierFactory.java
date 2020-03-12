package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.alicamodel.Quantifier;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.HashMap;

public class QuantifierFactory extends Factory {

    public static Quantifier create (Element quantifierNode) {
        Quantifier quantifier = new Quantifier();
        String quantifierType = quantifierNode.getAttribute(Tags.XSITYPE);
        if (quantifierType.equals(Tags.ALLAGENTSTAG)) {
            quantifier.setQuantifierType(Types.QUANTIFIER_FORALL);
        } else {
            throw new RuntimeException("[QuantifierFactory] Unknown quantifier type: '" + quantifierType +"'");
        }
        Factory.setAttributes(quantifierNode, quantifier);
        conversionTool.planElements.put(quantifier.getId(), quantifier);

        Factory.quantifierScopeReferences.put (quantifier.getId(), Factory.getReferencedId(quantifierNode.getAttribute(Tags.SCOPE)));

        NodeList sortNodeList = quantifierNode.getElementsByTagName(Tags.SORTS);
        for (int i = 0; i < sortNodeList.getLength(); i++) {
            Element sortNode = (Element) sortNodeList.item(i);
            quantifier.addSort(sortNode.getTextContent());
        }

        return quantifier;
    }

    public static void attachReferences() {
        for (HashMap.Entry<Long, Long> entry : Factory.quantifierScopeReferences.entrySet()) {
            Quantifier quantifier = (Quantifier) conversionTool.planElements.get(entry.getKey());
            PlanElement planElement = conversionTool.planElements.get(entry.getValue());
            quantifier.setScope(planElement);
        }
        Factory.quantifierScopeReferences.clear();
    }
}
