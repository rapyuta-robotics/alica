package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.State;
import de.unikassel.vs.alica.planDesigner.alicamodel.Synchronisation;
import de.unikassel.vs.alica.planDesigner.alicamodel.Transition;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.HashMap;

public class TransitionFactory extends Factory {
    public static Transition create(Element transitionNode, ConversionProcess cp) {
        Transition transition = new Transition();
        Factory.setAttributes(transitionNode, transition);
        cp.addElement(transition);
        cp.transitionInStateReferences.put(transition.getId(),  cp.getReferencedId(transitionNode.getElementsByTagName(Tags.INSTATE).item(0).getTextContent()));
        cp.transitionOutStateReferences.put(transition.getId(),  cp.getReferencedId(transitionNode.getElementsByTagName(Tags.OUTSTATE).item(0).getTextContent()));
        NodeList synchronisationsList = transitionNode.getElementsByTagName(Tags.SYNCHRONISATION);
        if (synchronisationsList.getLength() > 0 ) {
            cp.transitionSynchReferences.put(transition.getId(), cp.getReferencedId(synchronisationsList.item(0).getTextContent()));
        }

        NodeList preConditionList = transitionNode.getElementsByTagName(Tags.PRECONDITION);
        if (preConditionList.getLength() > 0) {
            transition.setPreCondition(PreConditionFactory.create((Element) preConditionList.item(0), cp));
        }

        return transition;
    }

    public static void attachReferences(ConversionProcess cp) {
        ConditionFactory.attachReferences(cp);

        for (HashMap.Entry<Long, Long> entry : cp.transitionOutStateReferences.entrySet()) {
            Transition transition = (Transition) cp.getElement(entry.getKey());
            State state = (State) cp.getElement(entry.getValue());
            transition.setOutState(state);
        }
        cp.transitionOutStateReferences.clear();

        for (HashMap.Entry<Long, Long> entry : cp.transitionInStateReferences.entrySet()) {
            Transition transition = (Transition) cp.getElement(entry.getKey());
            State state = (State) cp.getElement(entry.getValue());
            transition.setInState(state);
        }
        cp.transitionInStateReferences.clear();

        for (HashMap.Entry<Long, Long> entry : cp.transitionSynchReferences.entrySet()) {
            Transition transition = (Transition) cp.getElement(entry.getKey());
            Synchronisation synchronisation = (Synchronisation) cp.getElement(entry.getValue());
            transition.setSynchronisation(synchronisation);
        }
        cp.transitionSynchReferences.clear();
    }
}
