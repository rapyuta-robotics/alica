package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Synchronisation;
import de.unikassel.vs.alica.planDesigner.alicamodel.Transition;
import javafx.util.Pair;
import org.w3c.dom.Element;

public class SynchronisationFactory extends Factory {

    public static Synchronisation create(Element synchronisationNode, ConversionProcess cp) {
        Synchronisation synchronisation = new Synchronisation();
        Factory.setAttributes(synchronisationNode, synchronisation);
        cp.addElement(synchronisation);
        synchronisation.setFailOnSyncTimeout(Boolean.parseBoolean(synchronisationNode.getAttribute(Tags.FAILONSYNCTIMEOUT)));
        synchronisation.setSyncTimeout(Integer.parseInt(synchronisationNode.getAttribute(Tags.SYNCTIMEOUT)));
        synchronisation.setTalkTimeout(Integer.parseInt(synchronisationNode.getAttribute(Tags.TALKTIMEOUT)));

        String synchedTransitionsString = synchronisationNode.getAttribute(Tags.SYNCHEDTRANSITIONS);
        String[] transitionIdStrings = synchedTransitionsString.split(" ");
        for (String transitionIdString : transitionIdStrings) {
            cp.synchTransitionReferences.put(synchronisation.getId(), cp.getReferencedId(transitionIdString));
        }

        return synchronisation;
    }

    public static void attachReferences(ConversionProcess cp) {
        for (Pair<Long, Long> entry : cp.synchTransitionReferences.getEntries()) {
            Synchronisation synchronisation = (Synchronisation) cp.getElement(entry.getKey());
            Transition transition = (Transition) cp.getElement(entry.getValue());
            if (transition == null) {
                throw new RuntimeException("[SynchronisationFactory] Transition with ID " + entry.getValue() + " unknown!");
            }
            synchronisation.addSyncedTransition(transition);
        }
        cp.synchTransitionReferences.clear();
    }
}
