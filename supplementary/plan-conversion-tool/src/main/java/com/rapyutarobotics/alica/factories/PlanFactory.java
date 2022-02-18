package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.ArrayList;

public class PlanFactory extends Factory {
    public static Plan create(Element planNode, ConversionProcess cp) {
        Plan plan = new Plan();
        Factory.setAttributes(planNode, plan);
        cp.addElement(plan);
        AbstractPlanFactory.setVariables(planNode, plan, cp);

        plan.setMasterPlan(Boolean.parseBoolean(planNode.getAttribute(Tags.MASTERPLAN)));
        plan.setUtilityThreshold(Double.parseDouble(planNode.getAttribute(Tags.UTILITYTHRESHOLD)));
        ArrayList<EntryPoint> entryPoints = EntryPointFactory.create(planNode.getElementsByTagName(Tags.ENTRYPOINTS), cp);
        for (EntryPoint entryPoint : entryPoints) {
            plan.addEntryPoint(entryPoint);
        }

        NodeList stateNodes = planNode.getElementsByTagName(Tags.STATES);
        for (int i = 0; i < stateNodes.getLength(); i++) {
            Element stateNode = (Element) stateNodes.item(i);
            if (stateNode.getAttribute(Tags.XSITYPE).equals(Tags.SUCCESSSTATETAG)
                    || stateNode.getAttribute(Tags.XSITYPE).equals(Tags.FAILURESTATETAG)) {
                TerminalState terminalState = TerminalStateFactory.create(stateNode, cp);
                terminalState.setParentPlan(plan);
                plan.addState(terminalState);
            } else {
                State state = StateFactory.create(stateNode, cp);
                state.setParentPlan(plan);
                plan.addState(state);
            }
        }

        NodeList transitionsNodeList = planNode.getElementsByTagName(Tags.TRANSITIONS);
        for (int i = 0; i < transitionsNodeList.getLength(); i++) {
            Element transitionNode = (Element) transitionsNodeList.item(i);
            plan.addTransition(TransitionFactory.create(transitionNode, cp));
        }

        NodeList conditionsList = planNode.getElementsByTagName(Tags.CONDITIONS);
        for (int i = 0; i < conditionsList.getLength(); i++) {
            Element conditionNode = (Element) conditionsList.item(i);
            if (!conditionNode.getParentNode().getNodeName().equals(Tags.PLANTAG)) {
                // filter for those <preConditions..> nodes that are direct children of <alica:plan..> nodes
                continue;
            }

            if (conditionNode.getAttribute(Tags.XSITYPE).equals(Tags.PRECONDITIONTAG)) {
                plan.setPreCondition(PreConditionFactory.create(conditionNode, cp));
            } else if (conditionNode.getAttribute(Tags.XSITYPE).equals(Tags.RUNTIMECONDITIONTAG)) {
                plan.setRuntimeCondition(RuntimeConditionFactory.create(conditionNode, cp));
            } else {
                throw new RuntimeException("[PlanFactory] Unknown condition type '"
                        + conditionNode.getAttribute(Tags.XSITYPE) + "' found!");
            }
        }

        NodeList synchronisationsList = planNode.getElementsByTagName(Tags.SYNCHRONISATIONS);
        for (int i = 0; i < synchronisationsList.getLength(); i++) {
            Element synchronisationNode = (Element) synchronisationsList.item(i);
            Synchronisation synchronisation = SynchronisationFactory.create(synchronisationNode, cp);
            synchronisation.setPlan(plan);
            plan.addSynchronisation(synchronisation);
        }

        return plan;
    }

    public static void attachReferences(ConversionProcess cp) {
        EntryPointFactory.attachReferences(cp);
        StateFactory.attachReferences(cp);
        TransitionFactory.attachReferences(cp);
        SynchronisationFactory.attachReferences(cp);
        ConditionFactory.attachReferences(cp);
    }
}
