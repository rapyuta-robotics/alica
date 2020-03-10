package com.rapyutarobotics.alica;

import com.rapyutarobotics.alica.factories.*;
import de.unikassel.vs.alica.planDesigner.alicamodel.Behaviour;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import org.w3c.dom.Element;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.FactoryConfigurationError;
import java.io.File;
import java.nio.file.Paths;
import java.util.LinkedList;

public class ConversionTool {

    private ModelManager modelManager;
    private DocumentBuilderFactory documentBuilderFactory;

    public LinkedList<String> filesParsed;
    public LinkedList<String> filesToParse;

    public ConversionTool() {
        this.modelManager = new ModelManager();
        this.documentBuilderFactory = DocumentBuilderFactory.newDefaultInstance();
        this.filesParsed = new LinkedList<>();
        this.filesToParse = new LinkedList<>();
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
        modelManager.setPlansPath(Paths.get(args[3], "plans").toString());
        modelManager.setRolesPath(Paths.get(args[3], "roles").toString());
        modelManager.setTasksPath(Paths.get(args[3], "tasks").toString());

        // start parsing given master plan
        loadPlanTree(args[4]);
    }

    public void loadPlanTree(String pathToMasterPlan) {
        filesParsed.add(pathToMasterPlan);
        parseFile(pathToMasterPlan, Types.PLAN);
        while (!filesToParse.isEmpty()) {
            String fileToParse = filesToParse.poll();

            System.out.println("[ConversionTool] fileToParse: " + fileToParse);

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

        // TODO
//        this->attachReferences();
//        this->generateTemplateVariables();
//        this->computeReachabilities();

        for (Behaviour beh : modelManager.getBehaviours()) {
            System.out.println("[ConversionTool] " + beh.toString());
        }
    }

    public PlanElement parseFile(String currentFile, String type) {
        Element node;
        try {
            DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();
            node = documentBuilder.parse(currentFile).getDocumentElement();
        } catch (Exception e) {
            // noop
            return null;
        }

        if (Types.PLAN.equals(type)) {
            return PlanFactory.create(node);
        } else if (Types.BEHAVIOUR.equals(type)) {
            return BehaviourFactory.create(node);
        } else if (Types.PLANTYPE.equals(type)) {
            return PlanTypeFactory.create(node);
        } else if (Types.TASKREPOSITORY.equals(type)) {
            return TaskRepositoryFactory.create(node);
        } else if (Types.ROLESET.equals(type)) {
            return RoleSetFactory.create(node);
        } else {
            System.err.println("[ConversionTool] Parsing type not handled: " + type);
            return null;
        }
    }

    public static void main(String[] args) {
        ConversionTool conversionTool = new ConversionTool();
        conversionTool.convert(args);
    }
}