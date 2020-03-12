package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.HashMap;

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

    public static void attachReferences() {
        for (HashMap.Entry<Long, Long> entry : Factory.bindingSubPlanReferences.entrySet()) {
            VariableBinding variableBinding = (VariableBinding) conversionTool.planElements.get(entry.getKey());
            Plan plan = (Plan) conversionTool.planElements.get(entry.getValue());
            variableBinding.setSubPlan(plan);
        }
        Factory.bindingSubPlanReferences.clear();

        for (HashMap.Entry<Long, Long> entry : Factory.bindingSubVarReferences.entrySet()) {
            VariableBinding variableBinding = (VariableBinding) conversionTool.planElements.get(entry.getKey());
            Variable variable = (Variable) conversionTool.planElements.get(entry.getValue());
            variableBinding.setSubVariable(variable);
        }
        Factory.bindingSubVarReferences.clear();

        for (HashMap.Entry<Long, Long> entry : Factory.bindingVarReferences.entrySet()) {
            VariableBinding variableBinding = (VariableBinding) conversionTool.planElements.get(entry.getKey());
            Variable variable = (Variable) conversionTool.planElements.get(entry.getValue());
            variableBinding.setVariable(variable);
        }
        Factory.bindingVarReferences.clear();
    }
}
