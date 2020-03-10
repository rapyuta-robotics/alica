package com.rapyutarobotics.alica.factories;

import de.unikassel.vs.alica.planDesigner.alicamodel.TerminalState;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class TerminalStateFactory extends Factory {
    public static TerminalState create(Element terminalStateNode) {
        TerminalState terminalState;

        if (terminalStateNode.getAttribute(XSITYPE) == XSISUCCESSSTATE) {
            terminalState = new TerminalState(true);
        } else if (terminalStateNode.getAttribute(XSITYPE) == XSIFAILURESTATE) {
            terminalState = new TerminalState(false);
        } else {
            throw new RuntimeException("[TerminalStateFactory] Unknown terminal state type!");
        }
        Factory.setAttributes(terminalStateNode, terminalState);
        conversionTool.planElements.put(terminalState.getId(), terminalState);
        // TODO postcondition

        NodeList inTransitionNodes = terminalStateNode.getElementsByTagName(INTRANSITIONS);
        for (int i = 0; i  < inTransitionNodes.getLength(); i++) {
            Element inTransitionNode = (Element) inTransitionNodes.item(i);
            Factory.stateInTransitionReferences.put(terminalState.getId(), Factory.getReferencedId(inTransitionNode.getTextContent()));
        }

        return terminalState;
    }
}
