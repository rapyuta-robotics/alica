package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.VariableBinding;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class VariableBindingFactory extends Factory {
    public static VariableBinding create (Element variableBindingNode) {
        VariableBinding variableBinding = new VariableBinding();
        Factory.setAttributes(variableBindingNode, variableBinding);
        conversionTool.planElements.put(variableBinding.getId(), variableBinding);
        NodeList variableNodeList = variableBindingNode.getElementsByTagName(Tags.VARIABLE);
        if (variableNodeList.getLength() > 0) {
            Factory.bindingVarReferences.put(variableBinding.getId(), Factory.getReferencedId(variableNodeList.item(0).getTextContent()));
        }
        NodeList subPlanNodeList = variableBindingNode.getElementsByTagName(Tags.SUBPLAN);
        if (subPlanNodeList.getLength() > 0) {
            Factory.bindingSubPlanReferences.put(variableBinding.getId(), Factory.getReferencedId(variableNodeList.item(0).getTextContent()));
        }
        NodeList subVariableList = variableBindingNode.getElementsByTagName(Tags.SUBVARIABLE);
        if (subVariableList.getLength() > 0) {
            Factory.bindingSubVarReferences.put(variableBinding.getId(), Factory.getReferencedId(variableNodeList.item(0).getTextContent()));
        }
        return variableBinding;
    }
}
