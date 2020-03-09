package com.rapyutarobotics.alica.factories;

import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import org.w3c.dom.Element;

public class PlanFactory extends Factory{

    public static Plan create(Element node) {
        System.out.println("getTagName: " + node.getTagName());
        System.out.println("getBaseURI: " + node.getBaseURI());
        System.out.println("getLocalName: " + node.getLocalName());
        System.out.println("getNodeName: " + node.getNodeName());

        Plan plan = new Plan(Long.parseLong(node.getAttribute("id")));
        plan.setName(node.getAttribute("name"));
        plan.setMasterPlan(Boolean.parseBoolean(node.getAttribute("masterPlan") ));
        plan.setUtilityThreshold(Double.parseDouble(node.getAttribute("utilityThreshold")));
        plan.setComment(node.getAttribute("comment"));
        // TODO: extract relative directory from baseURI or something like that
        plan.setRelativeDirectory("");

        Factory.modelManager.storePlanElement(Types.PLAN, plan, true);
        return plan;
    }
}
