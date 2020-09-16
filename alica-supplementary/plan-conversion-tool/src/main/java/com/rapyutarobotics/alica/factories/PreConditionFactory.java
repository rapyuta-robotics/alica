package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.PreCondition;
import org.w3c.dom.Element;

public class PreConditionFactory extends Factory {

    public static PreCondition create(Element preConditionNode, ConversionProcess cp) {
        PreCondition preCondition = new PreCondition();
        ConditionFactory.fillConditon(preConditionNode, preCondition, cp);
        preCondition.setEnabled(Boolean.parseBoolean(preConditionNode.getAttribute(Tags.ENABLED)));
        return preCondition;
    }
}
