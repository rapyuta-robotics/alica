package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.Variable;
import de.unikassel.vs.alica.planDesigner.alicamodel.VariableBinding;
import javafx.util.Pair;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class VariableBindingFactory extends Factory {
    public static VariableBinding create (Element variableBindingNode, ConversionProcess cp) {
        VariableBinding variableBinding = new VariableBinding();
        Factory.setAttributes(variableBindingNode, variableBinding);
        cp.addElement(variableBinding);
        NodeList variableNodeList = variableBindingNode.getElementsByTagName(Tags.VARIABLE);
        if (variableNodeList.getLength() > 0) {
            cp.bindingVarReferences.put(variableBinding.getId(), cp.getReferencedId(variableNodeList.item(0).getTextContent()));
        }
        NodeList subPlanNodeList = variableBindingNode.getElementsByTagName(Tags.SUBPLAN);
        if (subPlanNodeList.getLength() > 0) {
            cp.bindingSubPlanReferences.put(variableBinding.getId(), cp.getReferencedId(variableNodeList.item(0).getTextContent()));
        }
        NodeList subVariableList = variableBindingNode.getElementsByTagName(Tags.SUBVARIABLE);
        if (subVariableList.getLength() > 0) {
            cp.bindingSubVarReferences.put(variableBinding.getId(), cp.getReferencedId(variableNodeList.item(0).getTextContent()));
        }
        return variableBinding;
    }

    public static void attachReferences(ConversionProcess cp) {
        for (Pair<Long, Long> entry : cp.bindingSubPlanReferences.getEntries()) {
            VariableBinding variableBinding = (VariableBinding) cp.getElement(entry.getKey());
            Plan plan = (Plan) cp.getElement(entry.getValue());
            if (plan == null) {
                throw new RuntimeException("[VariableBindingFactory] Sub plan with ID " + entry.getValue() + " unknown!");
            }
            variableBinding.setSubPlan(plan);
        }
        cp.bindingSubPlanReferences.clear();

        for (Pair<Long, Long> entry : cp.bindingSubVarReferences.getEntries()) {
            VariableBinding variableBinding = (VariableBinding) cp.getElement(entry.getKey());
            Variable variable = (Variable) cp.getElement(entry.getValue());
            if (variable == null) {
                throw new RuntimeException("[VariableBindingFactory] Sub variable with ID " + entry.getValue() + " unknown!");
            }
            variableBinding.setSubVariable(variable);
        }
        cp.bindingSubVarReferences.clear();

        for (Pair<Long, Long> entry : cp.bindingVarReferences.getEntries()) {
            VariableBinding variableBinding = (VariableBinding) cp.getElement(entry.getKey());
            Variable variable = (Variable) cp.getElement(entry.getValue());
            if (variable == null) {
                throw new RuntimeException("[VariableBindingFactory] Variable with ID " + entry.getValue() + " unknown!");
            }
            variableBinding.setVariable(variable);
        }
        cp.bindingVarReferences.clear();
    }
}
