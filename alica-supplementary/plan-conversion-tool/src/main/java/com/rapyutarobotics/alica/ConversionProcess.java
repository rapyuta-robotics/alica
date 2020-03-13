package com.rapyutarobotics.alica;

import com.rapyutarobotics.alica.factories.*;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;
import javafx.util.Pair;
import org.w3c.dom.Element;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.io.FileNotFoundException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Scanner;

public class ConversionProcess {

    /**
     * Manager from the new Plan Designer.
     */
    private ModelManager modelManager;
    /**
     * Class for parsing XML files.
     */
    private DocumentBuilderFactory documentBuilderFactory;

    private LinkedList<String> filesParsed;
    private LinkedList<String> filesToParse;
    private HashMap<Long, PlanElement> planElements;

    private boolean inputDirectoriesSet;
    private String basePlansPath;
    private String baseRolesPath;
    private String baseTasksPath;

    private boolean outputDirectorySet;
    private String outputDirectory;

    public HashMap<Long, Long> epStateReferences = new HashMap<>();
    public HashMap<Long, Long> epTaskReferences = new HashMap<>();
    public HashMap<Long, Long> stateInTransitionReferences = new HashMap<>();
    public HashMap<Long, Long> stateOutTransitionReferences = new HashMap<>();
    public HashMap<Long, Long> stateAbstractPlanReferences = new HashMap<>();
    public HashMap<Long, Long> conditionVarReferences = new HashMap<>();
    public HashMap<Long, Long> quantifierScopeReferences = new HashMap<>();
    public HashMap<Long, Long> bindingVarReferences = new HashMap<>();
    public HashMap<Long, Long> bindingSubPlanReferences = new HashMap<>();
    public HashMap<Long, Long> bindingSubVarReferences = new HashMap<>();
    public HashMap<Long, Long> transitionInStateReferences = new HashMap<>();
    public HashMap<Long, Long> transitionOutStateReferences = new HashMap<>();
    public HashMap<Long, Long> transitionSynchReferences = new HashMap<>();
    public HashMap<Long, Long> synchTransitionReferences = new HashMap<>();
    public HashMap<Long, Long> annotedPlanPlanReferences = new HashMap<>();
    public HashMap<Long, Long> roleSetRoleReferences = new HashMap<>();
    public HashMap<Long, Pair<Long, Float>> roleTaskReferences = new HashMap<>();

    public ConversionProcess() {
        this.modelManager = new ModelManager();
        this.documentBuilderFactory = DocumentBuilderFactory.newDefaultInstance();
        this.filesParsed = new LinkedList<>();
        this.filesToParse = new LinkedList<>();
        this.planElements = new HashMap<>();
        this.inputDirectoriesSet = false;
        this.outputDirectorySet = false;
    }

    public void addElement(PlanElement planElement) {
        this.planElements.put(planElement.getId(), planElement);
    }

    public PlanElement getElement(long id) {
        return this.planElements.get(id);
    }

    /**
     * Set paths for parsing old plans, tasks, roles.
     *
     * @param basePlansPath
     * @param baseTasksPath
     * @param baseRolesPath
     */
    public void setInputDirectories(String basePlansPath, String baseTasksPath, String baseRolesPath) {
        this.basePlansPath = basePlansPath;
        this.baseTasksPath = baseTasksPath;
        this.baseRolesPath = baseRolesPath;
        this.inputDirectoriesSet = true;
    }

    /**
     * Considers the given directory as base path and
     * further creates a "plans", "roles", and "tasks" folder
     * in it.
     *
     * @param outputDirectory The base path for all further output directories.
     */
    public void setOutputDirectory(String outputDirectory) {
        if (outputDirectorySet) {
            System.out.println("[ConversionTool] Output directory was set before to '" + this.outputDirectory);
        }

        this.outputDirectory = outputDirectory;

        // set output directories: plans, roles, tasks
        Path tmpPath = Paths.get(outputDirectory, "plans");
        tmpPath.toFile().mkdirs();
        modelManager.setPlansPath(tmpPath.toString());

        tmpPath = Paths.get(outputDirectory, "roles");
        tmpPath.toFile().mkdirs();
        modelManager.setRolesPath(tmpPath.toString());

        tmpPath = Paths.get(outputDirectory, "tasks");
        tmpPath.toFile().mkdirs();
        modelManager.setTasksPath(tmpPath.toString());

        this.outputDirectorySet = true;
    }

    /**
     * Extracts the ID in a given reference string and
     * adds the referenced file to the files that needs to
     * be parsed, if it is not already in that list.
     *
     * @param referenceString The reference string to be investigated
     * @return The id inside the reference string
     */
    public Long getReferencedId(String referenceString) {
        int idxOfHashtag = referenceString.lastIndexOf('#');
        if (idxOfHashtag == -1) {
            return Long.parseLong(referenceString);
        }
        String locator = referenceString.substring(0, idxOfHashtag);
        if (!locator.isEmpty()) {
            String fileReferenced = "";
            if (locator.endsWith(Extensions.PLAN) || locator.endsWith(Extensions.BEHAVIOUR) || locator.endsWith(Extensions.PLANTYPE)) {
                fileReferenced = Paths.get(this.basePlansPath, locator).normalize().toString();
            } else if (locator.endsWith(Extensions.TASKREPOSITORY)) {
                fileReferenced = Paths.get(this.baseTasksPath, locator).normalize().toString();
            } else if (locator.endsWith(Extensions.ROLESET)) {
                fileReferenced = Paths.get(this.baseRolesPath, locator).normalize().toString();
            } else if (locator.endsWith(".pp")) {
                System.out.println("[ConversionProcess] Info - Planning Problems are not supported anymore. Gonna ignore reference: '" + referenceString + "'");
            } else if (locator.endsWith(".rdefset")) {
                fileReferenced = Paths.get(this.baseRolesPath, locator).normalize().toString();
            } else {
                throw new RuntimeException("[ConversionProcess] Unknown file extension: " + locator);
            }

            if (!fileReferenced.isEmpty()
                    && !filesParsed.contains(fileReferenced)
                    && !filesToParse.contains(fileReferenced)) {
                filesToParse.add(fileReferenced);
            }
        }

        return Long.parseLong(referenceString.substring(idxOfHashtag + 1));
    }

    public void addTaskrepositoryToParseQueue(Long taskID) {
        String taskIDString = "" + taskID;
        File folder = new File(this.baseTasksPath);
        File[] listOfFiles = folder.listFiles();

        for (int i = 0; i < listOfFiles.length; i++) {
            if (listOfFiles[i].isDirectory()) {
                continue;
            }

            File file = listOfFiles[i];

            final Scanner scanner;
            try {
                scanner = new Scanner(file);
            } catch (FileNotFoundException e) {
                throw new RuntimeException("[RoleSetFactory] Error during search for Task Repository that includes task with ID '" + taskID + "'");
            }
            while (scanner.hasNextLine()) {
                final String lineFromFile = scanner.nextLine();
                if (lineFromFile.contains(taskIDString)) {

                }
            }
        }
    }

    /**
     * Given the start file for the conversion process, this method
     * triggers the following steps:
     * 1. parsing old xml-based files
     * 2. serialising new json-based files
     * 3. TODO: Autogenerated code conversion.
     *
     * @param masterFileToConvert The master plan or roleset to convert.
     */
    public void run(String masterFileToConvert) {
        if (!inputDirectoriesSet) {
            throw new RuntimeException("[ConversionProcess] Please first specify the input directories for plans, tasks, and roles before starting the conversion.");
        }

        if (!outputDirectorySet) {
            throw new RuntimeException("[ConversionProcess] Please first specify the base output directory before starting the conversion.");
        }

        // parses given master file and attaches references in a final step
        this.load(masterFileToConvert);

        // serialises the parsed serializable elements
        this.serialise();
    }

    /**
     * Start the parsing of the given file and loops over
     * other files, found via references in the given and
     * other files, until all files are parsed. Finally references
     * between the files are resolved to the actual objects.
     *
     * @param masterFileToConvert The start file for the parsing.
     */
    private void load(String masterFileToConvert) {
        // init queue
        String fileToParse = masterFileToConvert;
        fileToParse = new File(fileToParse).toPath().normalize().toString();
        filesToParse.add(fileToParse);

        // process until all found/referenced files are done
        while (!filesToParse.isEmpty()) {
            fileToParse = filesToParse.poll();

            File f = new File(fileToParse);
            if (!f.exists() || f.isDirectory()) {
                throw new RuntimeException("[ConversionTool] Cannot find referenced file:" + fileToParse);
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
            } else if (fileToParse.endsWith("rset")) {
                // new ending in new plan designer is ".rst"
                System.out.println("[ConversionProcess] Info - New file ending for role sets is '*.rst'. Gonna rename file: " + fileToParse);
                parseFile(fileToParse, Types.ROLESET);
            } else if (fileToParse.endsWith("rdefset")) {
                // new ending in new plan designer is ".rst"
                System.out.println("[ConversionProcess] Info - Role Definition Sets (*.rdefset) are included in Role Sets (*.rst) in the new Plan Designer. Gonna integrate '" + fileToParse + "'");
                parseFile(fileToParse, "RoleDefinitionSet");
            } else {
                throw new RuntimeException("[ConversionTool] Cannot parse file type: " + fileToParse);
            }
        }

        // resolve found references
        this.attachReferences();
    }

    /**
     * Parses the given file and meanwhile stores referenced files into
     * 'filesToParse' list. This method triggers the Factory according
     * to the given type.
     *
     * @param currentFile The file to be parsed.
     * @param type        The object type that is represented in the given file.
     */
    private void parseFile(String currentFile, String type) {
        Element node;
        try {
            DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();
            node = documentBuilder.parse(currentFile).getDocumentElement();
        } catch (Exception e) {
            // noop
            return;
        }
        ArrayList<SerializablePlanElement> elements = new ArrayList<>();
        if (Types.PLAN.equals(type)) {
            elements.add(PlanFactory.create(node, this));
        } else if (Types.BEHAVIOUR.equals(type)) {
            elements = BehaviourFactory.create(node, this);
        } else if (Types.PLANTYPE.equals(type)) {
            elements.add(PlanTypeFactory.create(node, this));
        } else if (Types.TASKREPOSITORY.equals(type)) {
            elements.add(TaskRepositoryFactory.create(node, this));
        } else if (Types.ROLESET.equals(type)) {
            elements.add(RoleSetFactory.create(node, this));
        } else if ("RoleDefinitionSet".equals(type)) {
            // does not exist in the new plan designer format
            RoleFactory.create(node, this);
        } else {
            throw new RuntimeException("[ConversionTool] Parsing type not handled: " + type);
        }

        for (SerializablePlanElement element : elements) {
            this.setRelativeDirectory(element, currentFile);
        }
    }

    /**
     * Determines the relative directory of the given serializable plan
     * element. This information is necessary for the new Plan Designer.
     *
     * @param element      Serializable plan element, whose relative directory should be set.
     * @param absoluteFile The absolute file that helps to determine the relative directory.
     */
    private void setRelativeDirectory(SerializablePlanElement element, String absoluteFile) {
        int baseLength = 0;
        if (absoluteFile.endsWith(Extensions.PLAN) || absoluteFile.endsWith(Extensions.BEHAVIOUR) || absoluteFile.endsWith(Extensions.PLANTYPE)) {
            baseLength = this.basePlansPath.length();
        } else if (absoluteFile.endsWith(Extensions.TASKREPOSITORY)) {
            baseLength = this.baseTasksPath.length();
        } else if (absoluteFile.endsWith("rset")) {
            // ending changed for new plan designer
            baseLength = this.baseRolesPath.length();
        } else {
            throw new RuntimeException("[ConversionProcess] Unable to create relative directory from '" + absoluteFile + "'");
        }

        int lastSeparatorIdx = absoluteFile.lastIndexOf(File.separatorChar);
        String relativeDirectory = absoluteFile.substring(baseLength, lastSeparatorIdx);
        if (!relativeDirectory.isEmpty() && relativeDirectory.charAt(0) == File.separatorChar) {
            relativeDirectory = relativeDirectory.substring(1);
        }
        element.setRelativeDirectory(relativeDirectory);
    }

    /**
     * Calls recursively factories to resolve references
     * to the actual objects, parsed before. Note: After the
     * resolving, the reference maps are cleared. Though,
     * calling this method more than once, won't have any effect.
     */
    private void attachReferences() {
        RoleSetFactory.attachReferences(this);
        PlanFactory.attachReferences(this);
        PlanTypeFactory.attachReferences(this);
        BehaviourFactory.attachReferences(this);
    }

    /**
     * Serialises all model objects parsed from XML format
     * into files in the JSON format.
     */
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
//                System.out.println("[ConversionTool] Info - Skip " + element.toString());
            }
        }
    }
}
