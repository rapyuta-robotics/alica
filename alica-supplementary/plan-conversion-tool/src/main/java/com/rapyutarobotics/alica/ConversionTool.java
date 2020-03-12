package com.rapyutarobotics.alica;

import com.rapyutarobotics.alica.factories.*;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import org.w3c.dom.Element;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

public class ConversionTool {

    private ModelManager modelManager;
    private DocumentBuilderFactory documentBuilderFactory;

    public LinkedList<String> filesParsed;
    public LinkedList<String> filesToParse;
    public HashMap<Long, PlanElement> planElements;

    public ConversionTool() {
        this.modelManager = new ModelManager();
        this.documentBuilderFactory = DocumentBuilderFactory.newDefaultInstance();
        this.filesParsed = new LinkedList<>();
        this.filesToParse = new LinkedList<>();
        this.planElements = new HashMap<>();
        Factory.setModelManager(this.modelManager);
        Factory.setConversionTool(this);
    }

    public void convert(String[] args) {
        if (args.length < 5) {
            System.err.println("Usage: java -jar ConversionTool(..).jar <plans-path> <roles-path> <tasks-path> <output-directory> <path-to-MasterPlan.pml>");
            return;
        }

        // set paths for parsing old plans, roles, tasks
        Factory.basePlansPath = args[0];
        Factory.baseRolesPath = args[1];
        Factory.baseTasksPath = args[2];
        Factory.setConversionTool(this);

        // set output directories
        Path tmpPath = Paths.get(args[3], "plans");
        tmpPath.toFile().mkdirs();
        modelManager.setPlansPath(tmpPath.toString());

        tmpPath = Paths.get(args[3], "roles");
        tmpPath.toFile().mkdirs();
        modelManager.setRolesPath(tmpPath.toString());

        tmpPath = Paths.get(args[3], "tasks");
        tmpPath.toFile().mkdirs();
        modelManager.setTasksPath(tmpPath.toString());

        // start parsing given master plan
        loadPlanTree(args[4]);

        // start serialising of parsed plan elements
        serialise();
    }

    private void serialise() {
        for (PlanElement element : this.planElements.values()) {
            if (element instanceof Plan) {
                this.modelManager.storePlanElement(Types.PLAN, element, true);
            } else if (element instanceof Behaviour) {
                this.modelManager.storePlanElement(Types.BEHAVIOUR, element, true);
            } else if (element instanceof PlanType) {
                this.modelManager.storePlanElement(Types.PLANTYPE, element, true);
            } else if (element instanceof TaskRepository) {
                this.modelManager.storePlanElement(Types.TASKREPOSITORY, element, true);
            } else if (element instanceof RoleSet) {
                this.modelManager.storePlanElement(Types.ROLESET, element, true);
            } else {
                System.out.println("[ConversionTool] Skip " + element.toString());
            }
        }
    }

    private void loadPlanTree(String pathToMasterPlan) {
        filesParsed.add(pathToMasterPlan);
        String fileToParse = pathToMasterPlan;
        fileToParse = new File(fileToParse).toPath().normalize().toString();
        parseFile(fileToParse, Types.PLAN);
        while (!filesToParse.isEmpty()) {
            fileToParse = filesToParse.poll();
//            System.out.println("[ConversionTool] fileToParse: " + fileToParse);

            File f = new File(fileToParse);
            if(!f.exists() || f.isDirectory()) {
                System.err.println("[ConversionTool] Cannot find referenced file:" + fileToParse);
                return;
            }
            filesParsed.add(fileToParse);
            if (fileToParse.endsWith(Extensions.PLAN)) {
                parseFile(fileToParse, Types.PLAN);
            } else if (fileToParse.endsWith(Extensions.TASKREPOSITORY)) {
                parseFile(fileToParse, Types.TASKREPOSITORY);
            } else if (fileToParse.endsWith(Extensions.BEHAVIOUR)) {
                parseFile(fileToParse, Types.BEHAVIOUR);
            } else if (fileToParse.endsWith(Extensions.PLANTYPE)) {
                parseFile(fileToParse, Types.PLANTYPE);
            } else {
                System.err.println("[ConversionTool] Cannot parse file type: " + fileToParse);
            }
        }

        this.attachReferences();

        for (Behaviour beh : modelManager.getBehaviours()) {
            System.out.println("[ConversionTool] " + beh.toString());
        }
    }

    private void loadRoleSet(String pathToRoleSet) {
        // TODO
//        std::string roleSetPath;
//        if (!essentials::FileSystem::findFile(this->baseRolePath, roleSetName + alica::Strings::roleset_extension, roleSetPath)) {
//            roleSetPath = findDefaultRoleSet(baseRolePath);
//        }
//
//        if (!essentials::FileSystem::pathExists(roleSetPath)) {
//            AlicaEngine::abort("MM: Cannot find RoleSet '" + roleSetPath + "'");
//        }
//
//        RoleSet* roleSet = (RoleSet*) parseFile(roleSetPath, alica::Strings::roleset);
//        RoleSetFactory::attachReferences();
//        ALICA_INFO_MSG("MM: Parsed the following role set: \n" << roleSet->toString());
//        return roleSet;

        RoleSetFactory.attachReferences();
    }

    private void attachReferences() {
        // these methods call recursively other factories for the job
        PlanFactory.attachReferences();
        PlanTypeFactory.attachReferences();
        BehaviourFactory.attachReferences();
    }

    public ArrayList<SerializablePlanElement> parseFile(String currentFile, String type) {
        Element node;
        try {
            DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();
            node = documentBuilder.parse(currentFile).getDocumentElement();
        } catch (Exception e) {
            // noop
            return null;
        }

        ArrayList<SerializablePlanElement> serializablePlanElements = new ArrayList<>();
        if (Types.PLAN.equals(type)) {
            Plan plan = PlanFactory.create(node);
            System.out.println("[ConversionTool] PLAN Relative Path '" + getRelativeDirectory(currentFile, Types.PLAN) + "'");
            plan.setRelativeDirectory(getRelativeDirectory(currentFile, Types.PLAN));
            serializablePlanElements.add(plan);
        } else if (Types.BEHAVIOUR.equals(type)) {
            ArrayList<Behaviour> behaviours = BehaviourFactory.create(node);
            for (Behaviour behaviour : behaviours) {
                System.out.println("[ConversionTool] BEHAVIOUR Relative Path '" + getRelativeDirectory(currentFile, Types.BEHAVIOUR) + "'");
                behaviour.setRelativeDirectory(getRelativeDirectory(currentFile, Types.BEHAVIOUR));
                serializablePlanElements.add(behaviour);
            }
        } else if (Types.PLANTYPE.equals(type)) {
            PlanType planType =  PlanTypeFactory.create(node);
            System.out.println("[ConversionTool] PLANTYPE Relative Path '" + getRelativeDirectory(currentFile, Types.PLANTYPE) + "'");
            planType.setRelativeDirectory(getRelativeDirectory(currentFile, Types.PLANTYPE));
            serializablePlanElements.add(planType);
        } else if (Types.TASKREPOSITORY.equals(type)) {
            TaskRepository taskRepository = TaskRepositoryFactory.create(node);
            System.out.println("[ConversionTool] TASKREPOSITORY Relative Path '" + getRelativeDirectory(currentFile, Types.TASKREPOSITORY) + "'");
            taskRepository.setRelativeDirectory(getRelativeDirectory(currentFile, Types.TASKREPOSITORY));
            serializablePlanElements.add(taskRepository);
        } else if (Types.ROLESET.equals(type)) {
            RoleSet roleSet = RoleSetFactory.create(node);
            System.out.println("[ConversionTool] ROLESET Relative Path '" + getRelativeDirectory(currentFile, Types.ROLESET) + "'");
            roleSet.setRelativeDirectory(getRelativeDirectory(currentFile, Types.ROLESET));
            serializablePlanElements.add(roleSet);
        } else if ("RoleDefinitionSet".equals(type)) {
            // does not exist in the new plan designer format
            RoleFactory.create(node);
            return null;
        } else {
            System.err.println("[ConversionTool] Parsing type not handled: " + type);
            return null;
        }
        return serializablePlanElements;
    }

    public String getRelativeDirectory (String absoluteFile, String type) {
        int baseLength = 0;
        if (Types.PLAN.equals(type)||Types.BEHAVIOUR.equals(type)||Types.PLANTYPE.equals(type)) {
            baseLength = Factory.basePlansPath.length();
        } else if (Types.TASKREPOSITORY.equals(type)) {
            baseLength = Factory.baseTasksPath.length();
        } else if (Types.ROLESET.equals(type)) {
            baseLength = Factory.baseRolesPath.length();
        } else {
            System.err.println("[ConversionTool] Parsing type not handled: " + type);
            return "";
        }

        int lastSeparatorIdx = absoluteFile.lastIndexOf(File.separatorChar);
        String relativeDirectory = absoluteFile.substring(baseLength, lastSeparatorIdx);
        if(!relativeDirectory.isEmpty() && relativeDirectory.charAt(0) == File.separatorChar) {
            relativeDirectory = relativeDirectory.substring(1);
        }
        return relativeDirectory;
    }

    public static void main(String[] args) {
        ConversionTool conversionTool = new ConversionTool();
        conversionTool.convert(args);
    }
}