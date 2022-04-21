package de.unikassel.vs.alica.generator;

import de.unikassel.vs.alica.generator.plugin.IPlugin;
import de.unikassel.vs.alica.generator.plugin.PluginManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.IOException;

/**
 * This Codegenerator console application (re)generates all plans/behaviours and terminates afterwards.
 */
public class StandaloneCodegenerator {
    private static final Logger LOG = LogManager.getLogger(StandaloneCodegenerator.class);

    private static String clangFormatPath;
    private static String sourceGenPath;
    private static String plansPath;
    private static String rolesPath;
    private static String tasksPath;
    private static String pluginsPath;
    private static String packageName = "lbc";

    private static void printUsage(){
        System.out.println("Usage: java -jar StandaloneCodegenerator <clangFormat> <sourceGenPath> <plansPath> <rolesPath> <tasksPaths> <pluginsPath> [PackageName - default is \"lbc\"]");
        System.exit(-1);
    }

    private static void readCmdLineArgs(String[] args) {
        if (args.length < 6)
        {
            printUsage();
        }

        clangFormatPath = args[0];
        sourceGenPath = args[1];
        plansPath = args[2];
        rolesPath = args[3];
        tasksPath = args[4];
        pluginsPath = args[5];
        if (args.length >= 7) {
            packageName = args[6];
        }
    }

    public static void main(String[] args) throws IOException {
        System.out.println("Starting code generation...");
        readCmdLineArgs(args);

        PluginManager.getInstance().updateAvailablePlugins(pluginsPath);
        PluginManager.getInstance().setDefaultPlugin("DefaultPlugin");

        ModelManager modelManager = new ModelManager();
        modelManager.setPlansPath(plansPath);
        modelManager.setTasksPath(tasksPath);
        modelManager.setRolesPath(rolesPath);
        modelManager.loadModelFromDisk();

        GeneratedSourcesManager generatedSourcesManager = new GeneratedSourcesManager(packageName);
        generatedSourcesManager.setCodegenPath(sourceGenPath);

        Codegenerator codegenerator = new Codegenerator(modelManager.getPlans(),
                modelManager.getBehaviours(),
                modelManager.getConditions(),
                clangFormatPath,
                generatedSourcesManager,
                packageName);

        codegenerator.generate();
        System.out.println("Success!");
    }
}
