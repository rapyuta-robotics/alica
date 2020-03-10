package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionTool;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import org.w3c.dom.Element;

import java.lang.reflect.Field;
import java.nio.file.Paths;
import java.util.HashMap;

public class Factory {
    static ModelManager modelManager;
    static ConversionTool conversionTool;

    public static String basePlansPath;
    public static String baseRolesPath;
    public static String baseTasksPath;

    static HashMap<Long, Long> epStateReferences = new HashMap<>();
    static HashMap<Long, Long> epTaskReferences = new HashMap<>();
    static HashMap<Long, Long> stateInTransitionReferences = new HashMap<>();
    static HashMap<Long, Long> stateOutTransitionReferences = new HashMap<>();
    static HashMap<Long, Long> stateAbstractPlanReferences = new HashMap<>();

    // Attribute Names in XML
    static String ID = "id";
    static String NAME = "name";
    static String PLANS = "plans";
    static String PLANTAG = "alica:Plan";
    static String MASTERPLAN = "masterPlan";
    static String UTILITYTHRESHOLD = "utilityThreshold";
    static String COMMENT = "comment";
    static String VARIABLES = "vars";
    static String VARIABLETYPE = "Type";
    static String ENTRYPOINTS = "entryPoints";
    static String MINCARDINALITY = "minCardinality";
    static String MAXCARDINALITY = "maxCardinality";
    static String SUCCESSREQUIRED = "successRequired";
    static String STATE = "state";
    static String STATES = "states";
    static String XSISUCCESSSTATE = "alica:SuccessState";
    static String XSIFAILURESTATE = "alica::FailureState";
    static String TASK = "task";
    static String XSITYPE = "xsi:type";
    static String INTRANSITIONS = "inTransitions";
    static String OUTTRANSITIONS = "outTransitions";


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

    public static void setModelManager(ModelManager modelManager) {
        Factory.modelManager = modelManager;
    }
    public static void setConversionTool(ConversionTool conversionTool) { Factory.conversionTool = conversionTool; }

    public static void setAttributes(Element node, PlanElement element) {
        try {
            idField.set(element, Long.parseLong(node.getAttribute(ID)));
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        element.setName(node.getAttribute(NAME));
        element.setComment(node.getAttribute(COMMENT));
    }

    public static Long getReferencedId(String idString) {
        int idxOfHashtag = idString.lastIndexOf('#');
        if (idxOfHashtag == -1) {
            return Long.parseLong(idString);
        }
        String locator = idString.substring(0, idxOfHashtag);
        if (!locator.isEmpty()) {
            String fileReferenced = "";
            if (locator.endsWith(Extensions.PLAN) ||locator.endsWith(Extensions.BEHAVIOUR) || locator.endsWith(Extensions.PLANTYPE)) {
                fileReferenced = Paths.get(Factory.basePlansPath, locator).toString();
            } else if (locator.endsWith(Extensions.TASKREPOSITORY)) {
                fileReferenced = Paths.get(Factory.baseTasksPath, locator).toString();
            } else if (locator.endsWith(Extensions.ROLESET)) {
                fileReferenced = Paths.get(Factory.baseRolesPath, locator).toString();
            } else {
                System.out.println("[Factory] Unknown file extension: " + locator);
            }

            if (!conversionTool.filesParsed.contains(fileReferenced) && !conversionTool.filesToParse.contains(fileReferenced)) {
                conversionTool.filesToParse.add(fileReferenced);
            }
        }

        return Long.parseLong(idString.substring(idxOfHashtag + 1));
    }

    public static void storeElement(PlanElement element, String type)
    {
        // insert into general element map
        if (modelManager.getPlanElement(element.getId()) != null) {
            throw new RuntimeException("[Factory] ERROR: ID utilised twice: " + element.getId()
                    + "\n" + "ELEMENT >" + element.getName() + "< >"
                    + modelManager.getPlanElement(element.getId()).getName() + "<");
        }
        modelManager.storePlanElement(type, element, false);
    }


}
