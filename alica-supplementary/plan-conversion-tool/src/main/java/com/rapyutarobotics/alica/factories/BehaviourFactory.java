package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Behaviour;
import de.unikassel.vs.alica.planDesigner.alicamodel.SerializablePlanElement;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.ArrayList;

public class BehaviourFactory extends Factory {
    public static ArrayList<SerializablePlanElement> create(Element behaviourNode, ConversionProcess cp) {
        ArrayList<SerializablePlanElement> behaviours = new ArrayList<>();
        NodeList configurationNodes = behaviourNode.getElementsByTagName(Tags.CONFIGURATIONS);
        for (int i = 0; i < configurationNodes.getLength(); i++) {
            Element configuration = (Element) configurationNodes.item(i);
            if (i > 0) {
                System.out.println("[BehaviourFactory] Info - Multiple Behaviour Configurations are not supported anymore. Dropping the configuration with ID: " + configuration.getAttribute(Tags.ID));
                continue;
            }
            Behaviour behaviour = new Behaviour();
            Factory.setAttributes(behaviourNode, behaviour);
            cp.addElement(behaviour);
            System.out.println("[BehaviourFactory] Info - Behaviour Configurations are not supported anymore. Variables, frequency, deferring, and parameters are taken from configuration with ID: " + configuration.getAttribute(Tags.ID));
            AbstractPlanFactory.setVariables(configuration, behaviour, cp);
            behaviour.setFrequency(Integer.parseInt(configuration.getAttribute(Tags.FREQUENCY)));
            behaviour.setDeferring(Long.parseLong(configuration.getAttribute(Tags.DEFERRING)));
            // TODO parameters...

            behaviours.add(behaviour);
        }
        return behaviours;
    }

    public static void attachReferences(ConversionProcess cp){
        ConditionFactory.attachReferences(cp);
    }
}