package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanType;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class PlanTypeFactory extends Factory {
    public static PlanType create(Element planTypeNode) {
        PlanType planType = new PlanType();
        Factory.setAttributes(planTypeNode, planType);
        conversionTool.planElements.put(planType.getId(), planType);
        AbstractPlanFactory.setVariables(planTypeNode,planType);

        NodeList annotatedPlansList = planTypeNode.getElementsByTagName(Tags.PLANS);
        for (int i = 0; i < annotatedPlansList.getLength(); i++) {
            Element annotatedPlanNode = (Element) annotatedPlansList.item(i);
            planType.addAnnotatedPlan(AnnotatedPlanFactory.create(annotatedPlanNode));
        }

        NodeList variableBindingNodes = planTypeNode.getElementsByTagName(Tags.VARIABLEBINDING);
        for (int i = 0; i < variableBindingNodes.getLength(); i++) {
            Element variableBindingNode = (Element) variableBindingNodes.item(i);
            planType.addVariableBinding(VariableBindingFactory.create(variableBindingNode));
        }

        return planType;
    }
}