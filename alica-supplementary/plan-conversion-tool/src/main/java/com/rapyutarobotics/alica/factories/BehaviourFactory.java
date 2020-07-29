package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Behaviour;
import de.unikassel.vs.alica.planDesigner.alicamodel.Configuration;
import de.unikassel.vs.alica.planDesigner.alicamodel.SerializablePlanElement;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class BehaviourFactory extends Factory {
    public static SerializablePlanElement create(Element behaviourNode, ConversionProcess cp) {
        Behaviour behaviour = new Behaviour();
        Factory.setAttributes(behaviourNode, behaviour);
        cp.addElement(behaviour);

        NodeList configurationNodes = behaviourNode.getElementsByTagName(Tags.CONFIGURATIONS);
        for (int i = 0; i < configurationNodes.getLength(); i++) {
            Element configurationNode = (Element) configurationNodes.item(i);
            Configuration configuration = (Configuration) ConfigurationFactory.create(configurationNode, cp);
            if (i == 0) {
                System.out.println("[BehaviourFactory] Info - Behaviour Configurations are not supported anymore. Variables, frequency, deferring, and eventDriven are taken from configuration with ID: " + configuration.getId());
                AbstractPlanFactory.setVariables(configurationNode, behaviour, cp);
                behaviour.setFrequency(Integer.parseInt(configurationNode.getAttribute(Tags.FREQUENCY)));
                behaviour.setDeferring(Long.parseLong(configurationNode.getAttribute(Tags.DEFERRING)));
                behaviour.setEventDriven(Boolean.parseBoolean(configurationNode.getAttribute(Tags.EVENTDRIVEN)));
            }
            cp.configurationBehaviourMapping.put(configuration.getId(), behaviour.getId());
        }

        return behaviour;
    }

    public static void attachReferences(ConversionProcess cp){
        ConditionFactory.attachReferences(cp);
    }
}