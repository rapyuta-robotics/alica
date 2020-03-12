package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Role;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

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
}
