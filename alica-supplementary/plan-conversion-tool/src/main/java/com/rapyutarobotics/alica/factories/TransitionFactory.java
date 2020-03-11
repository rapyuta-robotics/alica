package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Transition;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class TransitionFactory extends Factory {
    public static Transition create(Element transitionNode) {
        Transition transition = new Transition();
        Factory.setAttributes(transitionNode, transition);
        conversionTool.planElements.put(transition.getId(), transition);
        Factory.transitionInStateReferences.put(transition.getId(),  Factory.getReferencedId(transitionNode.getElementsByTagName(Tags.INSTATE).item(0).getTextContent()));
        Factory.transitionOutStateReferences.put(transition.getId(),  Factory.getReferencedId(transitionNode.getElementsByTagName(Tags.OUTSTATE).item(0).getTextContent()));
        NodeList synchronisationsList = transitionNode.getElementsByTagName(Tags.SYNCHRONISATION);
        if (synchronisationsList.getLength() > 0 ) {
            Factory.transitionSynchReferences.put(transition.getId(), Factory.getReferencedId(synchronisationsList.item(0).getTextContent()));
        }

        NodeList preConditionList = transitionNode.getElementsByTagName(Tags.PRECONDITION);
        if (preConditionList.getLength() > 0) {
            transition.setPreCondition(PreConditionFactory.create((Element) preConditionList.item(0)));
        }

        return transition;
    }
}
