package com.rapyutarobotics.alica.factories;

import de.unikassel.vs.alica.planDesigner.alicamodel.PostCondition;
import org.w3c.dom.Element;

public class PostConditionFactory extends Factory {

    public static PostCondition create(Element postConditionNode) {
        PostCondition postCondition = new PostCondition();
        ConditionFactory.fillConditon(postConditionNode, postCondition);
        return postCondition;
    }
}
