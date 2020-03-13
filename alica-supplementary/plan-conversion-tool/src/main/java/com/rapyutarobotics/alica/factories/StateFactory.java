package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.HashMap;

public class StateFactory extends Factory {

    public static State create(Element stateNode, ConversionProcess cp) {
        State state = new State();
        Factory.setAttributes(stateNode, state);
        cp.addElement(state);
        NodeList inTransitionNodes = stateNode.getElementsByTagName(Tags.INTRANSITIONS);
        for (int i = 0; i  < inTransitionNodes.getLength(); i++) {
            Element inTransitionNode = (Element) inTransitionNodes.item(i);
            cp.stateInTransitionReferences.put(state.getId(), cp.getReferencedId(inTransitionNode.getTextContent()));
        }
        NodeList outTransitionNodes = stateNode.getElementsByTagName(Tags.OUTTRANSITIONS);
        for (int i = 0; i  < outTransitionNodes.getLength(); i++) {
            Element outTransitionNode = (Element) outTransitionNodes.item(i);
            cp.stateOutTransitionReferences.put(state.getId(), cp.getReferencedId(outTransitionNode.getTextContent()));
        }
        NodeList abstractPlanNodes = stateNode.getElementsByTagName(Tags.PLANS);
        for (int i = 0; i  < abstractPlanNodes.getLength(); i++) {
            Element abstractPlanNode = (Element) abstractPlanNodes.item(i);
            cp.stateAbstractPlanReferences.put(state.getId(), cp.getReferencedId(abstractPlanNode.getTextContent()));
        }
        NodeList variableBindingNodes = stateNode.getElementsByTagName(Tags.VARIABLEBINDING);
        for (int i = 0; i < variableBindingNodes.getLength(); i++) {
            Element variableBindingNode = (Element) variableBindingNodes.item(i);
            state.addVariableBinding(VariableBindingFactory.create(variableBindingNode, cp));
        }

        return state;
    }

    public static void attachReferences(ConversionProcess cp) {
        VariableBindingFactory.attachReferences(cp);

        for (HashMap.Entry<Long, Long> entry : cp.stateInTransitionReferences.entrySet()) {
            State state = (State) cp.getElement(entry.getKey());
            Transition transition = (Transition) cp.getElement(entry.getValue());
            state.addInTransition(transition);
        }
        cp.stateInTransitionReferences.clear();

        for (HashMap.Entry<Long, Long> entry : cp.stateOutTransitionReferences.entrySet()) {
            State state = (State) cp.getElement(entry.getKey());
            Transition transition = (Transition) cp.getElement(entry.getValue());
            state.addOutTransition(transition);
        }
        cp.stateOutTransitionReferences.clear();

        for (HashMap.Entry<Long, Long> entry : cp.stateAbstractPlanReferences.entrySet()) {
            State state = (State) cp.getElement(entry.getKey());
            AbstractPlan abstractPlan = (AbstractPlan) cp.getElement(entry.getValue());
            state.addAbstractPlan(abstractPlan);
        }
        cp.stateAbstractPlanReferences.clear();
    }
}
