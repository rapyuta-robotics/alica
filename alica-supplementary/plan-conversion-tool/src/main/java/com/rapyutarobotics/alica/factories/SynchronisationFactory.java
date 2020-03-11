package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Synchronisation;
import org.w3c.dom.Element;

public class SynchronisationFactory extends Factory {

    public static Synchronisation create(Element synchronisationNode) {
        Synchronisation synchronisation = new Synchronisation();
        Factory.setAttributes(synchronisationNode, synchronisation);
        conversionTool.planElements.put(synchronisation.getId(), synchronisation);
        synchronisation.setFailOnSyncTimeout(Boolean.parseBoolean(synchronisationNode.getAttribute(Tags.FAILONSYNCTIMEOUT)));
        synchronisation.setSyncTimeout(Integer.parseInt(synchronisationNode.getAttribute(Tags.SYNCTIMEOUT)));
        synchronisation.setTalkTimeout(Integer.parseInt(synchronisationNode.getAttribute(Tags.TALKTIMEOUT)));

        String synchedTransitionsString = synchronisationNode.getAttribute(Tags.SYNCHEDTRANSITIONS);
        String[] transitionIdStrings = synchedTransitionsString.split(" ");
        for (String transitionIdString : transitionIdStrings) {
            Factory.synchTransitionReferences.put(synchronisation.getId(), Factory.getReferencedId(transitionIdString));
        }

        return synchronisation;
    }
}
