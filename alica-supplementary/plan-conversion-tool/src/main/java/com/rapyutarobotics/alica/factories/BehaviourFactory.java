package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Behaviour;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.ArrayList;

public class BehaviourFactory extends Factory {
    public static ArrayList<Behaviour> create(Element behaviourNode) {
        ArrayList<Behaviour> behaviours = new ArrayList<>();
        NodeList configurationNodes = behaviourNode.getElementsByTagName(Tags.CONFIGURATIONS);
        for (int i = 0; i < configurationNodes.getLength(); i++) {
            Element configuration = (Element) configurationNodes.item(i);
            Behaviour behaviour = new Behaviour();
            Factory.setAttributes(configuration, behaviour);
            conversionTool.planElements.put(behaviour.getId(), behaviour);
            AbstractPlanFactory.setVariables(configuration, behaviour);
            behaviour.setFrequency(Integer.parseInt(configuration.getAttribute(Tags.FREQUENCY)));
            behaviour.setDeferring(Long.parseLong(configuration.getAttribute(Tags.DEFERRING)));
            // TODO parameters...

            behaviours.add(behaviour);
        }
        return behaviours;
    }
}