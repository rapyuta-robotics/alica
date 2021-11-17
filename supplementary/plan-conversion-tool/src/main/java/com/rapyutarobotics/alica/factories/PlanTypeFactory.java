package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.AnnotatedPlan;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanType;
import javafx.util.Pair;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class PlanTypeFactory extends Factory {
    public static PlanType create(Element planTypeNode, ConversionProcess cp) {
        PlanType planType = new PlanType();
        Factory.setAttributes(planTypeNode, planType);
        cp.addElement(planType);
        AbstractPlanFactory.setVariables(planTypeNode,planType, cp);

        NodeList annotatedPlansList = planTypeNode.getElementsByTagName(Tags.PLANS);
        for (int i = 0; i < annotatedPlansList.getLength(); i++) {
            Element annotatedPlanNode = (Element) annotatedPlansList.item(i);
            planType.addAnnotatedPlan(AnnotatedPlanFactory.create(annotatedPlanNode, cp));
        }

        NodeList variableBindingNodes = planTypeNode.getElementsByTagName(Tags.VARIABLEBINDING);
        for (int i = 0; i < variableBindingNodes.getLength(); i++) {
            Element variableBindingNode = (Element) variableBindingNodes.item(i);
            planType.addVariableBinding(VariableBindingFactory.create(variableBindingNode, cp));
        }

        return planType;
    }

    public static void attachReferences(ConversionProcess cp) {
        VariableBindingFactory.attachReferences(cp);

        for (Pair<Long, Long> entry : cp.annotedPlanPlanReferences.getEntries()) {
            AnnotatedPlan annotatedPlan = (AnnotatedPlan) cp.getElement(entry.getKey());
            Plan plan = (Plan) cp.getElement(entry.getValue());
            if (plan == null) {
                throw new RuntimeException("[PlanTypeFactory] Plan with ID " + entry.getValue() + " unknown!");
            }
            annotatedPlan.setPlan(plan);
        }
        cp.annotedPlanPlanReferences.clear();
    }
}