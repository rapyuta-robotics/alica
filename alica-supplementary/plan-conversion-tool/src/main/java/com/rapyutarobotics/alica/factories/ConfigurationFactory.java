package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Configuration;
import de.unikassel.vs.alica.planDesigner.alicamodel.SerializablePlanElement;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class ConfigurationFactory extends Factory {
    public static SerializablePlanElement create(Element configurationNode, ConversionProcess cp) {
        Configuration configuration = new Configuration();
        Factory.setAttributes(configurationNode, configuration);

        NodeList parameterNodes = configurationNode.getElementsByTagName(Tags.PARAMETERS);
        if (parameterNodes.getLength() > 0) {
            cp.addElement(configuration);
            for (int j = 0; j < parameterNodes.getLength(); j++) {
                Element parameter = (Element) parameterNodes.item(j);
                configuration.putParameter(parameter.getAttribute(Tags.KEY), parameter.getAttribute(Tags.VALUE));
            }
        }

        return configuration;
    }
}