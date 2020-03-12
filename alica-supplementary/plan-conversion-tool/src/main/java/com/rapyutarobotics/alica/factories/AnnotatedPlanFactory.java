package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.AnnotatedPlan;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class AnnotatedPlanFactory extends Factory {

    public static AnnotatedPlan create(Element annotatedPlanNode) {
        AnnotatedPlan annotatedPlan = new AnnotatedPlan();
        Factory.setAttributes(annotatedPlanNode, annotatedPlan);
        conversionTool.planElements.put(annotatedPlan.getId(), annotatedPlan);
        annotatedPlan.setActivated(Boolean.parseBoolean(annotatedPlanNode.getAttribute(Tags.ACTIVATED)));

        NodeList planNodeList = annotatedPlanNode.getElementsByTagName(Tags.PLAN);
        if (planNodeList.getLength() > 0) {
            Factory.annotedPlanPlanReferences.put(annotatedPlan.getId(), Factory.getReferencedId(planNodeList.item(0).getTextContent()));
        }

        return annotatedPlan;
    }
}
