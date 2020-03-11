package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.ArrayList;

public class PlanFactory extends Factory{
    public static Plan create(Element planNode) {
        Plan plan = new Plan();
        Factory.setAttributes(planNode, plan);
        conversionTool.planElements.put(plan.getId(), plan);
        AbstractPlanFactory.setVariables(planNode, plan);

        plan.setMasterPlan(Boolean.parseBoolean(planNode.getAttribute(Tags.MASTERPLAN)));
        plan.setUtilityThreshold(Double.parseDouble(planNode.getAttribute(Tags.UTILITYTHRESHOLD)));
        ArrayList<EntryPoint> entryPoints = EntryPointFactory.create(planNode.getElementsByTagName(Tags.ENTRYPOINTS));
        for (EntryPoint entryPoint : entryPoints) {
            plan.addEntryPoint(entryPoint);
        }

        NodeList stateNodes = planNode.getElementsByTagName(Tags.STATES);
        for (int i = 0; i < stateNodes.getLength(); i++) {
            Element stateNode = (Element) stateNodes.item(i);
            if (stateNode.getAttribute(Tags.XSITYPE).equals(Tags.SUCCESSSTATETAG)
                    || stateNode.getAttribute(Tags.XSITYPE).equals(Tags.FAILURESTATETAG)) {
                TerminalState terminalState = TerminalStateFactory.create(stateNode);
                terminalState.setParentPlan(plan);
                plan.addState(terminalState);
            } else {
                State state = StateFactory.create(stateNode);
                state.setParentPlan(plan);
                plan.addState(state);
            }
        }

        NodeList transitionsNodeList = planNode.getElementsByTagName(Tags.TRANSITIONS);
        for (int i = 0; i < transitionsNodeList.getLength(); i++) {
            Element transitionNode = (Element) transitionsNodeList.item(i);
            plan.addTransition(TransitionFactory.create(transitionNode));
        }

        NodeList conditionsList = planNode.getElementsByTagName(Tags.CONDITIONS);
        for (int i = 0; i < conditionsList.getLength(); i++) {
            Element conditionNode = (Element) conditionsList.item(i);
            if (!conditionNode.getParentNode().getNodeName().equals(Tags.PLANTAG)) {
                // filter for those <preConditions..> nodes that are direct children of <alica:plan..> nodes
                continue;
            }

            if (conditionNode.getAttribute(Tags.XSITYPE).equals(Tags.PRECONDITIONTAG)) {
                plan.setPreCondition(PreConditionFactory.create(conditionNode));
            } else if (conditionNode.getAttribute(Tags.XSITYPE).equals(Tags.RUNTIMECONDITIONTAG)){
                plan.setRuntimeCondition(RuntimeConditionFactory.create(conditionNode));
            } else {
                throw new RuntimeException("[PlanFactory] Unknown condition type '"
                        + conditionNode.getAttribute(Tags.XSITYPE) +  "' found!");
            }
        }

        NodeList synchronisationsList = planNode.getElementsByTagName(Tags.SYNCHRONISATIONS);
        for (int i = 0; i < synchronisationsList.getLength(); i++) {
            Element synchronisationNode = (Element) synchronisationsList.item(i);
            Synchronisation synchronisation = SynchronisationFactory.create(synchronisationNode);
            synchronisation.setPlan(plan);
            plan.addSynchronisation(synchronisation);
        }

        // TODO: extract relative directory from baseURI or something like that
        plan.setRelativeDirectory("");


        return plan;
    }
}
