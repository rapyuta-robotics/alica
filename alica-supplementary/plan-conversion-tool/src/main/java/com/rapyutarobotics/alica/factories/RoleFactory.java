package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Role;
import de.unikassel.vs.alica.planDesigner.alicamodel.Task;
import javafx.util.Pair;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.HashMap;

public class RoleFactory extends Factory {
    public static void create(Element roleDefinitionSetNode, ConversionProcess cp) {
        NodeList roleNodes = roleDefinitionSetNode.getElementsByTagName(Tags.ROLES);
        for (int i = 0; i < roleNodes.getLength(); i++) {
            Element roleNode = (Element) roleNodes.item(i);
            Role role = new Role();
            Factory.setAttributes(roleNode, role);
            cp.addElement(role);
        }
    }

    public static void attachReferences(ConversionProcess cp) {
        for (HashMap.Entry<Long, Pair<Long, Float>> entry : cp.roleTaskReferences.entrySet()) {
            Role role = (Role) cp.getElement(entry.getKey());
            Task task = (Task) cp.getElement(entry.getValue().getKey());
            if (task == null) {
                throw new RuntimeException("[RoleFactory] Did not find a Taskrepository that contains a task with ID '" + entry.getValue().getKey() + "', but it is referenced by priorities for role with ID '" + role.getId() + "'");
            }
            role.addTaskPriority(task, entry.getValue().getValue());
        }
        cp.roleTaskReferences.clear();
    }
}
