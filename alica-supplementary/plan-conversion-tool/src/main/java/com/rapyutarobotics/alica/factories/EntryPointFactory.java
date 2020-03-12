package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.ArrayList;
import java.util.HashMap;

public class EntryPointFactory extends Factory {

    public static ArrayList<EntryPoint> create(NodeList entryPointNodes) {
        ArrayList<EntryPoint> constructedEntryPoints = new ArrayList<>();
        for (int i = 0; i < entryPointNodes.getLength(); i++) {
            Element epNode = (Element) entryPointNodes.item(i);

            EntryPoint ep = new EntryPoint();
            Factory.setAttributes(epNode, ep);
            conversionTool.planElements.put(ep.getId(),ep);

            ep.setPlan((Plan) conversionTool.planElements.get(Long.parseLong(((Element) epNode.getParentNode()).getAttribute(Tags.ID))));
            ep.setMinCardinality(Integer.parseInt(epNode.getAttribute(Tags.MINCARDINALITY)));
            ep.setMaxCardinality(Integer.parseInt(epNode.getAttribute(Tags.MAXCARDINALITY)));
            ep.setSuccessRequired(Boolean.parseBoolean(epNode.getAttribute(Tags.SUCCESSREQUIRED)));
            Factory.epStateReferences.put(ep.getId(), Factory.getReferencedId(epNode.getElementsByTagName(Tags.STATE).item(0).getTextContent()));
            Factory.epTaskReferences.put(ep.getId(), Factory.getReferencedId(epNode.getElementsByTagName(Tags.TASK).item(0).getTextContent()));

            constructedEntryPoints.add(ep);
        }
        return constructedEntryPoints;
    }

    public static void attachReferences() {
        for (HashMap.Entry<Long, Long> entry : Factory.epTaskReferences.entrySet()) {
            EntryPoint entryPoint = (EntryPoint) conversionTool.planElements.get(entry.getKey());
            Task task = (Task) conversionTool.planElements.get(entry.getValue());
            entryPoint.setTask(task);
        }
        Factory.epTaskReferences.clear();

        for (HashMap.Entry<Long, Long> entry : Factory.epStateReferences.entrySet()) {
            EntryPoint entryPoint = (EntryPoint) conversionTool.planElements.get(entry.getKey());
            State state = (State) conversionTool.planElements.get(entry.getValue());
            entryPoint.setState(state);
        }
        Factory.epStateReferences.clear();
    }
}
