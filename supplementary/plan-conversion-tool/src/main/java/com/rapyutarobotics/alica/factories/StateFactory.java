package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import javafx.util.Pair;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

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
            state.addConfAbstractPlanWrapper(ConfAbstractPlanWrapperFactory.create(abstractPlanNode, cp));
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
        ConfAbstractPlanWrapperFactory.attachReferences(cp);

        for (Pair<Long, Long> entry : cp.stateInTransitionReferences.getEntries()) {
            State state = (State) cp.getElement(entry.getKey());
            Transition transition = (Transition) cp.getElement(entry.getValue());
            if (transition == null) {
                throw new RuntimeException("[StateFactory] Incoming transition with ID " + entry.getValue() + " unknown!");
            }
            state.addInTransition(transition);
        }
        cp.stateInTransitionReferences.clear();

        for (Pair<Long, Long> entry : cp.stateOutTransitionReferences.getEntries()) {
            State state = (State) cp.getElement(entry.getKey());
            Transition transition = (Transition) cp.getElement(entry.getValue());
            if (transition == null) {
                throw new RuntimeException("[StateFactory] Outgoing transition with ID " + entry.getValue() + " unknown!");
            }
            state.addOutTransition(transition);
        }
        cp.stateOutTransitionReferences.clear();
    }
}
