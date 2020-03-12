package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.HashMap;

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

    public static void attachReferences() {
        VariableBindingFactory.attachReferences();

        for (HashMap.Entry<Long, Long> entry : Factory.annotedPlanPlanReferences.entrySet()) {
            AnnotatedPlan annotatedPlan = (AnnotatedPlan) conversionTool.planElements.get(entry.getKey());
            Plan plan = (Plan) conversionTool.planElements.get(entry.getValue());
            annotatedPlan.setPlan(plan);
        }
        Factory.annotedPlanPlanReferences.clear();
    }
}