package com.rapyutarobotics.alica.factories;

import de.unikassel.vs.alica.planDesigner.alicamodel.EntryPoint;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.ArrayList;

public class PlanFactory extends Factory{

    public static Plan create(Element node) {
//        System.out.println("getTagName: " + node.getTagName());
//        System.out.println("getBaseURI: " + node.getBaseURI());
//        System.out.println("getLocalName: " + node.getLocalName());
//        System.out.println("getNodeName: " + node.getNodeName());

        Plan plan = new Plan();
        Factory.setAttributes(node, plan);
        AbstractPlanFactory.setVariables(node, plan);
        plan.setMasterPlan(Boolean.parseBoolean(node.getAttribute(MASTERPLAN)));
        plan.setUtilityThreshold(Double.parseDouble(node.getAttribute(UTILITYTHRESHOLD)));
        ArrayList<EntryPoint> entryPoints = EntryPointFactory.create(node.getElementsByTagName(ENTRYPOINTS));
        for (EntryPoint ep : entryPoints) {
            plan.addEntryPoint(ep);
        }

        // TODO: extract relative directory from baseURI or something like that
        plan.setRelativeDirectory("");

        Factory.modelManager.storePlanElement(Types.PLAN, plan, true);
        return plan;
    }
}
