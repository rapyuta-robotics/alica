package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.TerminalState;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class TerminalStateFactory extends Factory {
    public static TerminalState create(Element terminalStateNode, ConversionProcess cp) {
        TerminalState terminalState;

        if (terminalStateNode.getAttribute(Tags.XSITYPE).equals(Tags.SUCCESSSTATETAG)) {
            terminalState = new TerminalState(true);
        } else if (terminalStateNode.getAttribute(Tags.XSITYPE).equals(Tags.FAILURESTATETAG)) {
            terminalState = new TerminalState(false);
        } else {
            throw new RuntimeException("[TerminalStateFactory] Unknown terminal state type!");
        }
        Factory.setAttributes(terminalStateNode, terminalState);
        cp.addElement(terminalState);

        NodeList postConditionList = terminalStateNode.getElementsByTagName(Tags.POSTCONDITION);
        if (postConditionList.getLength() > 0) {
            // we only expect one postcondition on a terminal state
            terminalState.setPostCondition(PostConditionFactory.create((Element) postConditionList.item(0), cp));
        }

        NodeList inTransitionNodes = terminalStateNode.getElementsByTagName(Tags.INTRANSITIONS);
        for (int i = 0; i  < inTransitionNodes.getLength(); i++) {
            Element inTransitionNode = (Element) inTransitionNodes.item(i);
            cp.stateInTransitionReferences.put(terminalState.getId(), cp.getReferencedId(inTransitionNode.getTextContent()));
        }

        return terminalState;
    }
}
