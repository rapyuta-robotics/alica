package com.rapyutarobotics.alica.factories;

import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import org.w3c.dom.Element;

import java.lang.reflect.Field;
import java.util.Dictionary;
import java.util.HashMap;

public class Factory {

    static ModelManager modelManager;

    static HashMap<Long, Long> epStateReferences = new HashMap<>();
    static HashMap<Long, Long> epTaskReferences = new HashMap<>();

    // Attribute Names in XML
    static String ID = "id";
    static String NAME = "name";
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
    static String TASK = "task";

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

    public static void setAttributes(Element node, PlanElement element) {
        try {
            idField.set(element, node.getAttribute(ID));
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        element.setName(node.getAttribute(NAME));
        element.setComment(node.getAttribute(COMMENT));
    }

    public static Long getReferencedId(String idString) {
        // GO ON HERE...
//        std::size_t idxOfHashtag = idString.find_last_of("#");
//        if (idxOfHashtag == std::string::npos) {
//            return stoll(idString);
//        }
//        std::string locator = idString.substr(0, idxOfHashtag);
//        if (!locator.empty()) {
//            std::string fileReferenced;
//            if (essentials::FileSystem::endsWith(locator, alica::Strings::plan_extension) ||
//            essentials::FileSystem::endsWith(locator, alica::Strings::behaviour_extension) ||
//            essentials::FileSystem::endsWith(locator, alica::Strings::plantype_extension)) {
//                fileReferenced = essentials::FileSystem::combinePaths(modelManager->basePlanPath, locator);
//            } else if (essentials::FileSystem::endsWith(locator, alica::Strings::taskrepository_extension)) {
//                fileReferenced = essentials::FileSystem::combinePaths(modelManager->baseTaskPath, locator);
//            } else if (essentials::FileSystem::endsWith(locator, alica::Strings::roleset_extension)) {
//                fileReferenced = essentials::FileSystem::combinePaths(modelManager->baseRolePath, locator);
//            } else {
//                std::cout << "Factory: Unknown file extension: " << locator << std::endl;
//            }
//
//            if (std::find(std::begin(modelManager->filesParsed), std::end(modelManager->filesParsed), fileReferenced) == std::end(modelManager->filesParsed) &&
//                    std::find(std::begin(modelManager->filesToParse), std::end(modelManager->filesToParse), fileReferenced) ==
//            std::end(modelManager->filesToParse)) {
//                modelManager->filesToParse.push_back(fileReferenced);
//            }
//        }
//        return stoll(idString.substr(idxOfHashtag + 1, idString.size() - idxOfHashtag));
        return 0L;
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
