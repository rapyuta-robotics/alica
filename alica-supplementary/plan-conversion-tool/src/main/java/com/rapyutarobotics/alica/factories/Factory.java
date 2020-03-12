package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionTool;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import javafx.util.Pair;
import org.w3c.dom.Element;

import java.lang.reflect.Field;
import java.nio.file.Paths;
import java.util.HashMap;

public class Factory {
    public static ModelManager modelManager;
    public static ConversionTool conversionTool;

    public static String basePlansPath;
    public static String baseRolesPath;
    public static String baseTasksPath;

    public static HashMap<Long, Long> epStateReferences = new HashMap<>();
    public static HashMap<Long, Long> epTaskReferences = new HashMap<>();
    public static HashMap<Long, Long> stateInTransitionReferences = new HashMap<>();
    public static HashMap<Long, Long> stateOutTransitionReferences = new HashMap<>();
    public static HashMap<Long, Long> stateAbstractPlanReferences = new HashMap<>();
    public static HashMap<Long, Long> conditionVarReferences = new HashMap<>();
    public static HashMap<Long, Long> quantifierScopeReferences = new HashMap<>();
    public static HashMap<Long, Long> bindingVarReferences = new HashMap<>();
    public static HashMap<Long, Long> bindingSubPlanReferences = new HashMap<>();
    public static HashMap<Long, Long> bindingSubVarReferences = new HashMap<>();
    public static HashMap<Long, Long> transitionInStateReferences = new HashMap<>();
    public static HashMap<Long, Long> transitionOutStateReferences = new HashMap<>();
    public static HashMap<Long, Long> transitionSynchReferences = new HashMap<>();
    public static HashMap<Long, Long> synchTransitionReferences = new HashMap<>();
    public static HashMap<Long, Long> annotedPlanPlanReferences = new HashMap<>();
    public static HashMap<Long, Long> roleSetRoleReferences = new HashMap<>();
    public static HashMap<Long, Pair<Long, Float>> roleTaskReferences = new HashMap<>();

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
            idField.set(element, Long.parseLong(node.getAttribute(Tags.ID)));
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        element.setName(node.getAttribute(Tags.NAME));
        element.setComment(node.getAttribute(Tags.COMMENT));
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
                fileReferenced = Paths.get(Factory.basePlansPath, locator).normalize().toString();
            } else if (locator.endsWith(Extensions.TASKREPOSITORY)) {
                fileReferenced = Paths.get(Factory.baseTasksPath, locator).normalize().toString();
            } else if (locator.endsWith(Extensions.ROLESET)) {
                fileReferenced = Paths.get(Factory.baseRolesPath, locator).normalize().toString();
            } else {
                System.err.println("[Factory] Unknown file extension: " + locator);
            }

            if (!fileReferenced.isEmpty()
                    && !conversionTool.filesParsed.contains(fileReferenced)
                    && !conversionTool.filesToParse.contains(fileReferenced)) {
                conversionTool.filesToParse.add(fileReferenced);
            }
        }

        return Long.parseLong(idString.substring(idxOfHashtag + 1));
    }
}
