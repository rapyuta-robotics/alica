package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.AbstractPlan;
import de.unikassel.vs.alica.planDesigner.alicamodel.State;
import de.unikassel.vs.alica.planDesigner.alicamodel.Synchronisation;
import de.unikassel.vs.alica.planDesigner.alicamodel.Transition;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.HashMap;

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

    public static void attachReferences() {
        ConditionFactory.attachReferences();

        for (HashMap.Entry<Long, Long> entry : Factory.transitionOutStateReferences.entrySet()) {
            Transition transition = (Transition) conversionTool.planElements.get(entry.getKey());
            State state = (State) conversionTool.planElements.get(entry.getValue());
            transition.setOutState(state);
        }
        Factory.transitionOutStateReferences.clear();

        for (HashMap.Entry<Long, Long> entry : Factory.transitionInStateReferences.entrySet()) {
            Transition transition = (Transition) conversionTool.planElements.get(entry.getKey());
            State state = (State) conversionTool.planElements.get(entry.getValue());
            transition.setInState(state);
        }
        Factory.transitionInStateReferences.clear();

        for (HashMap.Entry<Long, Long> entry : Factory.transitionSynchReferences.entrySet()) {
            Transition transition = (Transition) conversionTool.planElements.get(entry.getKey());
            Synchronisation synchronisation = (Synchronisation) conversionTool.planElements.get(entry.getValue());
            transition.setSynchronisation(synchronisation);
        }
        Factory.transitionSynchReferences.clear();
    }
}
