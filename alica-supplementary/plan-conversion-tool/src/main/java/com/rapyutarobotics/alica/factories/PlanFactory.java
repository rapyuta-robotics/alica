package com.rapyutarobotics.alica.factories;

import de.unikassel.vs.alica.planDesigner.alicamodel.EntryPoint;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.State;
import de.unikassel.vs.alica.planDesigner.alicamodel.TerminalState;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.util.ArrayList;

public class PlanFactory extends Factory{
    public static Plan create(Element node) {
        Plan plan = new Plan();
        Factory.setAttributes(node, plan);
        conversionTool.planElements.put(plan.getId(), plan);
        AbstractPlanFactory.setVariables(node, plan);

        plan.setMasterPlan(Boolean.parseBoolean(node.getAttribute(MASTERPLAN)));
        plan.setUtilityThreshold(Double.parseDouble(node.getAttribute(UTILITYTHRESHOLD)));
        ArrayList<EntryPoint> entryPoints = EntryPointFactory.create(node.getElementsByTagName(ENTRYPOINTS));
        for (EntryPoint entryPoint : entryPoints) {
            plan.addEntryPoint(entryPoint);
        }

        NodeList stateNodes = node.getElementsByTagName(STATES);
        for (int i = 0; i < stateNodes.getLength(); i++) {
            Element stateNode = (Element) stateNodes.item(i);
            if (stateNode.getAttribute(XSITYPE) == XSISUCCESSSTATE || stateNode.getAttribute(XSITYPE) == XSIFAILURESTATE) {
                TerminalState terminalState = TerminalStateFactory.create(stateNode);
                terminalState.setParentPlan(plan);
                plan.addState(terminalState);
            } else {
                State state = StateFactory.create(stateNode);
                state.setParentPlan(plan);
                plan.addState(state);
            }
        }

        // TODO: extract relative directory from baseURI or something like that
        plan.setRelativeDirectory("");


        return plan;
    }
}
