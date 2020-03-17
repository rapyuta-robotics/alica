package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Role;
import de.unikassel.vs.alica.planDesigner.alicamodel.RoleSet;
import javafx.util.Pair;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.HashMap;

public class RoleSetFactory extends Factory {

    public static RoleSet create(Element roleSetNode, ConversionProcess cp) {
        RoleSet roleSet = new RoleSet();
        Factory.setAttributes(roleSetNode, roleSet);
        cp.addElement(roleSet);
        roleSet.setDefaultPriority(0.5f); // <- replacement for useableWithPlanID
        roleSet.setDefaultRoleSet(Boolean.parseBoolean(roleSetNode.getAttribute(Tags.DEFAULT)));

        NodeList rolePriorityMappings = roleSetNode.getElementsByTagName(Tags.MAPPINGS);
        for (int i = 0; i < rolePriorityMappings.getLength(); i++) {
            Element rolePriorityMapping = (Element) rolePriorityMappings.item(i);
            NodeList roleReferenceNodes = rolePriorityMapping.getElementsByTagName(Tags.ROLE);
            if (roleReferenceNodes.getLength() > 0) {
                Element roleReference = (Element) roleReferenceNodes.item(0);
                Long roleId = cp.getReferencedId(roleReference.getTextContent());
                cp.roleSetRoleReferences.put(roleSet.getId(), roleId);

                NodeList taskPriorities = rolePriorityMapping.getElementsByTagName(Tags.TASKPRIORITIES);
                for (int j = 0; j < taskPriorities.getLength(); j++) {
                    Element taskPriority = (Element) taskPriorities.item(j);
                    Long taskID = Long.parseLong(taskPriority.getAttribute(Tags.KEY));
                    // special treatment, because Role Sets don't reference task repositories in old Plan Designer
                    cp.addTaskrepositoryToParseQueue(taskID);
                    Float priority = Float.parseFloat(taskPriority.getAttribute(Tags.VALUE));
                    cp.roleTaskReferences.put(roleId, new Pair<Long, Float>(taskID, priority));
                }
            }
        }

        return roleSet;
    }



    public static void attachReferences(ConversionProcess cp) {
        RoleFactory.attachReferences(cp);

        for (HashMap.Entry<Long, Long> entry : cp.roleSetRoleReferences.entrySet()) {
            RoleSet roleSet = (RoleSet) cp.getElement(entry.getKey());
            Role role = (Role) cp.getElement(entry.getValue());
            roleSet.addRole(role);
            if (role == null) {
                throw new RuntimeException("[RoleSetFactory] Role with ID " + entry.getValue() + " unknown!");
            }
            role.setRoleSet(roleSet);
        }
        cp.roleSetRoleReferences.clear();
    }
}