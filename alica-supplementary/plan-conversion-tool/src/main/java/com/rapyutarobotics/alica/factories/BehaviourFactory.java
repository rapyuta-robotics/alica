package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Behaviour;
import de.unikassel.vs.alica.planDesigner.alicamodel.SerializablePlanElement;
import javafx.beans.property.BooleanProperty;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class BehaviourFactory extends Factory {
    public static SerializablePlanElement create(Element behaviourNode, ConversionProcess cp) {
        Behaviour behaviour = new Behaviour();
        Factory.setAttributes(behaviourNode, behaviour);
        cp.addElement(behaviour);

        NodeList configurationNodes = behaviourNode.getElementsByTagName(Tags.CONFIGURATIONS);
        for (int i = 0; i < configurationNodes.getLength(); i++) {
            Element configuration = (Element) configurationNodes.item(i);
            Long confID = Long.parseLong(configuration.getAttribute(Tags.ID));
            if (i == 0) {
                System.out.println("[BehaviourFactory] Info - Behaviour Configurations are not supported anymore. Variables, frequency, deferring, eventDriven, and parameters are taken from configuration with ID: " + confID);
                AbstractPlanFactory.setVariables(configuration, behaviour, cp);
                behaviour.setFrequency(Integer.parseInt(configuration.getAttribute(Tags.FREQUENCY)));
                behaviour.setDeferring(Long.parseLong(configuration.getAttribute(Tags.DEFERRING)));
                behaviour.setEventDriven(Boolean.parseBoolean(configuration.getAttribute(Tags.EVENTDRIVEN)));

                NodeList parameterNodes = configuration.getElementsByTagName(Tags.PARAMETERS);
                for (int j = 0; j < parameterNodes.getLength(); j++) {
                    Element parameter = (Element) parameterNodes.item(j);
                    behaviour.putParameter(parameter.getAttribute(Tags.KEY), parameter.getAttribute(Tags.VALUE));
                }
            } else {
                System.out.println("[BehaviourFactory] Info - Multiple Behaviour Configurations are not supported anymore. Dropping the configuration with ID: " + confID);
            }
            cp.configurationBehaviourMapping.put(confID, behaviour.getId());
        }

        return behaviour;
    }

    public static void attachReferences(ConversionProcess cp){
        ConditionFactory.attachReferences(cp);
    }
}