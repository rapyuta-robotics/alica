package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import de.unikassel.vs.alica.planDesigner.alicamodel.RuntimeCondition;
import org.w3c.dom.Element;

public class RuntimeConditionFactory extends Factory {

    public static RuntimeCondition create(Element runtimeConditionNode, ConversionProcess cp) {
        RuntimeCondition runtimeCondition = new RuntimeCondition();
        ConditionFactory.fillConditon(runtimeConditionNode, runtimeCondition, cp);
        return runtimeCondition;
    }
}
