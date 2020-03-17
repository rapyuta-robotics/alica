package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.EntryPoint;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.State;
import de.unikassel.vs.alica.planDesigner.alicamodel.Task;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.ArrayList;
import java.util.HashMap;

public class EntryPointFactory extends Factory {

    public static ArrayList<EntryPoint> create(NodeList entryPointNodes, ConversionProcess cp) {
        ArrayList<EntryPoint> constructedEntryPoints = new ArrayList<>();
        for (int i = 0; i < entryPointNodes.getLength(); i++) {
            Element epNode = (Element) entryPointNodes.item(i);

            EntryPoint ep = new EntryPoint();
            Factory.setAttributes(epNode, ep);
            cp.addElement(ep);

            ep.setPlan((Plan) cp.getElement(Long.parseLong(((Element) epNode.getParentNode()).getAttribute(Tags.ID))));
            ep.setMinCardinality(Integer.parseInt(epNode.getAttribute(Tags.MINCARDINALITY)));
            ep.setMaxCardinality(Integer.parseInt(epNode.getAttribute(Tags.MAXCARDINALITY)));
            ep.setSuccessRequired(Boolean.parseBoolean(epNode.getAttribute(Tags.SUCCESSREQUIRED)));
            cp.epStateReferences.put(ep.getId(), cp.getReferencedId(epNode.getElementsByTagName(Tags.STATE).item(0).getTextContent()));
            cp.epTaskReferences.put(ep.getId(), cp.getReferencedId(epNode.getElementsByTagName(Tags.TASK).item(0).getTextContent()));

            constructedEntryPoints.add(ep);
        }
        return constructedEntryPoints;
    }

    public static void attachReferences(ConversionProcess cp) {
        for (HashMap.Entry<Long, Long> entry : cp.epTaskReferences.entrySet()) {
            EntryPoint entryPoint = (EntryPoint) cp.getElement(entry.getKey());
            Task task = (Task) cp.getElement(entry.getValue());
            if (task == null) {
                throw new RuntimeException("[ConditionFactory] Task with ID " + entry.getValue() + " unknown!");
            }
            entryPoint.setTask(task);
        }
        cp.epTaskReferences.clear();

        for (HashMap.Entry<Long, Long> entry : cp.epStateReferences.entrySet()) {
            EntryPoint entryPoint = (EntryPoint) cp.getElement(entry.getKey());
            State state = (State) cp.getElement(entry.getValue());
            if (state == null) {
                throw new RuntimeException("[EntryPointFactory] State with ID " + entry.getValue() + " unknown!");
            }
            entryPoint.setState(state);
        }
        cp.epStateReferences.clear();
    }
}
