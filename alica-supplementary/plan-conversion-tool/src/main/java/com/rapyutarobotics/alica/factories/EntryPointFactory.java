package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.EntryPoint;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.util.ArrayList;

public class EntryPointFactory extends Factory {

    public static ArrayList<EntryPoint> create(NodeList entryPointNodes) {
        ArrayList<EntryPoint> constructedEntryPoints = new ArrayList<>();
        for (int i = 0; i < entryPointNodes.getLength(); i++) {
            Element epNode = (Element) entryPointNodes.item(i);

            EntryPoint ep = new EntryPoint();
            Factory.setAttributes(epNode, ep);
            Factory.storeElement(ep, Types.ENTRYPOINT);

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
}
