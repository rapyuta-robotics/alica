package com.rapyutarobotics.alica;

import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

public class ConversionTool {

    private ModelManager modelManager;
    private DocumentBuilderFactory documentBuilderFactory;

    public ConversionTool() {
        this.modelManager = new ModelManager();
        this.documentBuilderFactory = DocumentBuilderFactory.newDefaultInstance();
    }

    public void convert(String[] args) {
        if (args.length < 2) {
            System.err.println("Usage: java -jar ConversionTool(..).jar <output-directory> <path-to-MasterPlan.pml>");
            return;
        }
        modelManager.setPlansPath(args[0]);
        load(args[1]);
    }

    public void load(String pathToMasterPlan) {
        System.out.println("Loading: " + pathToMasterPlan);
        try {
            DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();
            Document doc = documentBuilder.parse(pathToMasterPlan);
            parsePlan(doc.getDocumentElement());
        } catch (Exception e) {
            // noop
        }
    }

    public void parsePlan(Element planNode) {
        System.out.println("getTagName: " + planNode.getTagName());
        System.out.println("getBaseURI: " + planNode.getBaseURI());
        System.out.println("getLocalName: " + planNode.getLocalName());
        System.out.println("getNodeName: " + planNode.getNodeName());

        Plan plan = new Plan(Long.parseLong(planNode.getAttribute("id")));
        plan.setName(planNode.getAttribute("name"));
        plan.setMasterPlan(Boolean.parseBoolean(planNode.getAttribute("masterPlan") ));
        plan.setUtilityThreshold(Double.parseDouble(planNode.getAttribute("utilityThreshold")));
        plan.setComment(planNode.getAttribute("comment"));
        // TODO: extract relative directory from baseURI or something like that
        plan.setRelativeDirectory("");

        modelManager.storePlanElement(Types.PLAN, plan, true);
    }

    public static void main(String[] args) {
        ConversionTool conversionTool = new ConversionTool();
        conversionTool.convert(args);
    }
}