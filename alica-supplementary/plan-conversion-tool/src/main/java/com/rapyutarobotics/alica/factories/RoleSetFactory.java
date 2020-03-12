package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.AnnotatedPlan;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.Role;
import de.unikassel.vs.alica.planDesigner.alicamodel.RoleSet;
import javafx.util.Pair;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.HashMap;

public class RoleSetFactory extends Factory {

    public static RoleSet create(Element roleSetNode) {
        RoleSet roleSet = new RoleSet();
        Factory.setAttributes(roleSetNode, roleSet);
        conversionTool.planElements.put(roleSet.getId(), roleSet);
        roleSet.setDefaultPriority(0.5f); // <- replacement for useableWithPlanID
        roleSet.setDefaultRoleSet(Boolean.parseBoolean(roleSetNode.getAttribute(Tags.DEFAULT)));

        NodeList rolePriorityMappings = roleSetNode.getElementsByTagName(Tags.MAPPINGS);
        for (int i = 0; i < rolePriorityMappings.getLength(); i++) {
            Element rolePriorityMapping = (Element) rolePriorityMappings.item(i);
            NodeList roleReferenceNodes = rolePriorityMapping.getElementsByTagName(Tags.ROLE);
            if (roleReferenceNodes.getLength() > 0) {
                Element roleReference = (Element) roleReferenceNodes.item(i);
                Long roleId = Factory.getReferencedId(roleReference.getTextContent());
                Factory.roleSetRoleReferences.put(roleSet.getId(), roleId);

                NodeList taskPriorities = rolePriorityMapping.getElementsByTagName(Tags.TASKPRIORITIES);
                for (int j = 0; j < taskPriorities.getLength(); j++) {
                    Element taskPriority = (Element) taskPriorities.item(j);
                    Long taskID = Long.parseLong(taskPriority.getAttribute(Tags.KEY));
                    Float priority = Float.parseFloat(taskPriority.getAttribute(Tags.VALUE));
                    Factory.roleTaskReferences.put(roleId, new Pair<Long, Float>(taskID, priority));
                }
            }
        }

        return roleSet;
    }

    public static void attachReferences() {
        RoleFactory.attachReferences();

        for (HashMap.Entry<Long, Long> entry : Factory.roleSetRoleReferences.entrySet()) {
            RoleSet roleSet = (RoleSet) conversionTool.planElements.get(entry.getKey());
            Role role = (Role) conversionTool.planElements.get(entry.getValue());
            roleSet.addRole(role);
        }
        Factory.roleSetRoleReferences.clear();
    }
}