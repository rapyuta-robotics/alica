package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.AnnotatedPlan;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class AnnotatedPlanFactory extends Factory {

    public static AnnotatedPlan create(Element annotatedPlanNode, ConversionProcess cp) {
        AnnotatedPlan annotatedPlan = new AnnotatedPlan();
        Factory.setAttributes(annotatedPlanNode, annotatedPlan);
        cp.addElement(annotatedPlan);
        annotatedPlan.setActivated(Boolean.parseBoolean(annotatedPlanNode.getAttribute(Tags.ACTIVATED)));

        NodeList planNodeList = annotatedPlanNode.getElementsByTagName(Tags.PLAN);
        if (planNodeList.getLength() > 0) {
            cp.annotedPlanPlanReferences.put(annotatedPlan.getId(), cp.getReferencedId(planNodeList.item(0).getTextContent()));
        }

        return annotatedPlan;
    }
}
