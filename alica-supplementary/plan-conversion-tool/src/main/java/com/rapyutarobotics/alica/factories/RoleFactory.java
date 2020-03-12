package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.alicamodel.Quantifier;
import de.unikassel.vs.alica.planDesigner.alicamodel.Role;
import de.unikassel.vs.alica.planDesigner.alicamodel.Task;
import javafx.util.Pair;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.HashMap;

public class RoleFactory extends Factory {
    public static void create(Element roleDefinitionSetNode) {
        NodeList roleNodes = roleDefinitionSetNode.getElementsByTagName(Tags.ROLES);
        for (int i = 0; i < roleNodes.getLength(); i++) {
            Element roleNode = (Element) roleNodes.item(i);
            Role role = new Role();
            Factory.setAttributes(roleNode, role);
            conversionTool.planElements.put(role.getId(), role);
        }
    }

    public static void attachReferences() {
        for (HashMap.Entry<Long, Pair<Long, Float>> entry : Factory.roleTaskReferences.entrySet()) {
            Role role = (Role) conversionTool.planElements.get(entry.getKey());
            Task task = (Task) conversionTool.planElements.get(entry.getValue().getKey());
            role.addTaskPriority(task, entry.getValue().getValue());
        }
        Factory.roleTaskReferences.clear();
    }
}
