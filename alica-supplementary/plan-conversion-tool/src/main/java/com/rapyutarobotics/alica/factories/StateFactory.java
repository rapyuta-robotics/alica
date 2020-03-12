package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.HashMap;

public class StateFactory extends Factory {

    public static State create(Element stateNode) {
        State state = new State();
        Factory.setAttributes(stateNode, state);
        conversionTool.planElements.put(state.getId(), state);
        NodeList inTransitionNodes = stateNode.getElementsByTagName(Tags.INTRANSITIONS);
        for (int i = 0; i  < inTransitionNodes.getLength(); i++) {
            Element inTransitionNode = (Element) inTransitionNodes.item(i);
            Factory.stateInTransitionReferences.put(state.getId(), Factory.getReferencedId(inTransitionNode.getTextContent()));
        }
        NodeList outTransitionNodes = stateNode.getElementsByTagName(Tags.OUTTRANSITIONS);
        for (int i = 0; i  < outTransitionNodes.getLength(); i++) {
            Element outTransitionNode = (Element) outTransitionNodes.item(i);
            Factory.stateOutTransitionReferences.put(state.getId(), Factory.getReferencedId(outTransitionNode.getTextContent()));
        }
        NodeList abstractPlanNodes = stateNode.getElementsByTagName(Tags.PLANS);
        for (int i = 0; i  < abstractPlanNodes.getLength(); i++) {
            Element abstractPlanNode = (Element) abstractPlanNodes.item(i);
            Factory.stateAbstractPlanReferences.put(state.getId(), Factory.getReferencedId(abstractPlanNode.getTextContent()));
        }
        NodeList variableBindingNodes = stateNode.getElementsByTagName(Tags.VARIABLEBINDING);
        for (int i = 0; i < variableBindingNodes.getLength(); i++) {
            Element variableBindingNode = (Element) variableBindingNodes.item(i);
            state.addVariableBinding(VariableBindingFactory.create(variableBindingNode));
        }

        return state;
    }

    public static void attachReferences() {
        VariableBindingFactory.attachReferences();

        for (HashMap.Entry<Long, Long> entry : Factory.stateInTransitionReferences.entrySet()) {
            State state = (State) conversionTool.planElements.get(entry.getKey());
            Transition transition = (Transition) conversionTool.planElements.get(entry.getValue());
            state.addInTransition(transition);
        }
        Factory.stateInTransitionReferences.clear();

        for (HashMap.Entry<Long, Long> entry : Factory.stateOutTransitionReferences.entrySet()) {
            State state = (State) conversionTool.planElements.get(entry.getKey());
            Transition transition = (Transition) conversionTool.planElements.get(entry.getValue());
            state.addOutTransition(transition);
        }
        Factory.stateOutTransitionReferences.clear();

        for (HashMap.Entry<Long, Long> entry : Factory.stateAbstractPlanReferences.entrySet()) {
            State state = (State) conversionTool.planElements.get(entry.getKey());
            AbstractPlan abstractPlan = (AbstractPlan) conversionTool.planElements.get(entry.getValue());
            state.addAbstractPlan(abstractPlan);
        }
        Factory.stateAbstractPlanReferences.clear();
    }
}
