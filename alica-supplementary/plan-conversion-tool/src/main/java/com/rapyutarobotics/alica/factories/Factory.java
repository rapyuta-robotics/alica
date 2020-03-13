package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.AnnotatedPlan;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import org.w3c.dom.Element;

import java.lang.reflect.Field;

public class Factory {

    // Reflection used to access the ID field of a PlanElement.
    // Note: This only works if we have the permission according to
    // the SecurityManager of the JVM.
    static Field idField;
    static {
        try {
            idField = PlanElement.class.getDeclaredField("id");
            idField.setAccessible(true);
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        }
    }

    public static void setAttributes(Element node, PlanElement element) {
        try {
            idField.set(element, Long.parseLong(node.getAttribute(Tags.ID)));
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        String name = node.getAttribute(Tags.NAME);
        if (element instanceof AnnotatedPlan) {
            System.out.println("[Factory] Info - Normalising the name of annotated plan (ID: " + element.getId() + ") to 'Annotated + <Name of Encapsulated Plan>'.");
        } else if (name.isEmpty()) {
            System.out.println("[Factory] Info - Some element has an empty name in the old XML format. Gonna replace it with its ID: " + element.getId());
        }
        element.setName(name);
        if (element.getName().equals(""+element.getId())) {
            System.out.println("[Factory] Info - The name '" + name + "' contains forbidden characters. Gonna replace it with the ID of the corresponding element: " + element.getId());
        }
        element.setComment(node.getAttribute(Tags.COMMENT));
    }
}
