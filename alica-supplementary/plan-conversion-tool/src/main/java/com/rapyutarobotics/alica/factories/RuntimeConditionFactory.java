package com.rapyutarobotics.alica.factories;

import de.unikassel.vs.alica.planDesigner.alicamodel.RuntimeCondition;
import org.w3c.dom.Element;

public class RuntimeConditionFactory extends Factory {

    public static RuntimeCondition create(Element runtimeConditionNode) {
        RuntimeCondition runtimeCondition = new RuntimeCondition();
        ConditionFactory.fillConditon(runtimeConditionNode, runtimeCondition);
        return runtimeCondition;
    }
}
