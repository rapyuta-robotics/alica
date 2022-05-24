package de.unikassel.vs.alica.generator.cpp;

import de.unikassel.vs.alica.generator.IConstraintCodeGenerator;
import de.unikassel.vs.alica.generator.IGenerator;
import de.unikassel.vs.alica.generator.plugin.PluginManager;
import de.unikassel.vs.alica.generator.GeneratedSourcesManager;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
/**
 * IF the following line is not import de.unikassel.vs.alica.generator.cpp.XtendTemplates;
 * you messed it up ... great ... you made the plandesigner great again ... huge ...
 * INSERT IT
 */
import de.unikassel.vs.alica.generator.cpp.XtendTemplates;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.*;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.FileAlreadyExistsException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

/**
 * Code generator for C++. It uses the XtendTemplates for creating the code.
 * After this the created strings are written to disk according to {@link GeneratedSourcesManager}.
 * Every file that is written is formatted by the formatter that is set by setFormatter.
 */
public class CPPGeneratorImpl implements IGenerator {

    private static final Logger LOG = LogManager.getLogger(CPPGeneratorImpl.class);
    private XtendTemplates xtendTemplates;

    private GeneratedSourcesManager generatedSourcesManager;
    private String formatter;
    private String packageName;

    public CPPGeneratorImpl(GeneratedSourcesManager generatedSourcesManager, String packageName) {
        this.generatedSourcesManager = generatedSourcesManager;
        this.packageName = packageName;
        xtendTemplates = new XtendTemplates();
    }

    /**
     * delegate XtendTemplates#setProtectedRegions(Map)
     *
     * @param protectedRegions mapping from identifier to content of protected region
     */
    @Override
    public void setProtectedRegions(Map<String, String> protectedRegions) {
        xtendTemplates.setProtectedRegions(protectedRegions);
    }

    @Override
    public void createBehaviourCreator(List<Behaviour> behaviours) {
        String headerPath = Paths.get(generatedSourcesManager.getIncludeDir(), "BehaviourCreator.h").toString();
        String fileContentHeader = xtendTemplates.behaviourCreatorHeader();
        writeSourceFile(headerPath, fileContentHeader);

        formatFile(headerPath);

        String srcPath = Paths.get(generatedSourcesManager.getSrcDir(), "BehaviourCreator.cpp").toString();
        String fileContentSource = xtendTemplates.behaviourCreatorSource(behaviours, packageName);
        writeSourceFile(srcPath, fileContentSource);

        formatFile(srcPath);
    }

    @Override
    public void createPlanCreator(List<Plan> plans) {
        String headerPath = Paths.get(generatedSourcesManager.getIncludeDir(), "PlanCreator.h").toString();
        String fileContentHeader = xtendTemplates.planCreatorHeader();
        writeSourceFile(headerPath, fileContentHeader);

        formatFile(headerPath);

        String srcPath = Paths.get(generatedSourcesManager.getSrcDir(), "PlanCreator.cpp").toString();
        String fileContentSource = xtendTemplates.planCreatorSource(plans, packageName);
        writeSourceFile(srcPath, fileContentSource);

        formatFile(srcPath);
    }

    @Override
    public void createBehaviour(Behaviour behaviour) {
        String destinationPath = cutDestinationPathToDirectory(behaviour);

        //DomainBehaviour
        String headerPath = Paths.get(generatedSourcesManager.getIncludeDir(), destinationPath, behaviour.getName() + behaviour.getId() + ".h").toString();
        String fileContentHeader = xtendTemplates.behaviourConditionHeader(behaviour, packageName);
        writeSourceFile(headerPath, fileContentHeader);

        formatFile(headerPath);

        String srcPath = Paths.get(generatedSourcesManager.getSrcDir(), destinationPath, behaviour.getName() + behaviour.getId() + ".cpp").toString();
        String fileContentSource = xtendTemplates.behaviourConditionSource(behaviour, getActiveConstraintCodeGenerator(), packageName);
        writeSourceFile(srcPath, fileContentSource);

        formatFile(srcPath);

        //Behaviour
        String headerPath2 = Paths.get(generatedSourcesManager.getIncludeDir(), destinationPath, behaviour.getName()+ ".h").toString();
        String fileContentHeader2 = xtendTemplates.behaviourHeader(behaviour, packageName);
        writeSourceFile(headerPath2, fileContentHeader2);

        formatFile(headerPath2);

        String srcPath2 = Paths.get(generatedSourcesManager.getSrcDir(), destinationPath, behaviour.getName()+ ".cpp").toString();
        String fileContentSource2 = xtendTemplates.behaviourSource(behaviour, packageName);
        writeSourceFile(srcPath2, fileContentSource2);

        formatFile(srcPath2);

//        useTemplateAndSaveResults(Paths.get(generatedSourcesManager.getSrcDir(), destinationPath, behaviour.getName() + ".cpp").toString(),
//                Paths.get(generatedSourcesManager.getIncludeDir(), destinationPath, behaviour.getName() + ".h").toString(),
//                behaviour,
//                xtendTemplates::behaviourHeader,
//                xtendTemplates::behaviourSource);
    }

    @Override
    public void createConditionCreator(List<Plan> plans, List<Behaviour> behaviours, List<Condition> conditions) {
        String headerPath = Paths.get(generatedSourcesManager.getIncludeDir(), "ConditionCreator.h").toString();
        String fileContentHeader = xtendTemplates.conditionCreatorHeader();
        writeSourceFile(headerPath, fileContentHeader);

        formatFile(headerPath);

        String srcPath = Paths.get(generatedSourcesManager.getSrcDir(), "ConditionCreator.cpp").toString();
        String fileContentSource = xtendTemplates.conditionCreatorSource(plans, behaviours, conditions, packageName);
        writeSourceFile(srcPath, fileContentSource);

        formatFile(srcPath);
    }

    /**
     * Small helper for writing source files
     *
     * @param filePath    filePath to write to
     * @param fileContent the content to write
     */
    private void writeSourceFile(String filePath, String fileContent) {
        try {

            if (Files.notExists(Paths.get(filePath).getParent())) {
                Files.createDirectories(Paths.get(filePath).getParent());
            }
            Files.write(Paths.get(filePath), fileContent.getBytes(StandardCharsets.UTF_8));
        } catch (IOException e) {
            LOG.error("Couldn't write source file "
                    + filePath + " with content size " + fileContent
                    .getBytes(StandardCharsets.UTF_8).length, e);
            throw new RuntimeException(e);
        }
    }

    @Override
    public void createConstraintCreator(List<Plan> plans, List<Behaviour> behaviours, List<Condition> conditions) {
        String headerPath = Paths.get(generatedSourcesManager.getIncludeDir(), "ConstraintCreator.h").toString();
        String fileContentHeader = xtendTemplates.constraintCreatorHeader();
        writeSourceFile(headerPath, fileContentHeader);

        formatFile(headerPath);

        String srcPath = Paths.get(generatedSourcesManager.getSrcDir(), "ConstraintCreator.cpp").toString();
        String fileContentSource = xtendTemplates.constraintCreatorSource(plans, behaviours, conditions, packageName);
        writeSourceFile(srcPath, fileContentSource);

        formatFile(srcPath);
    }

    /**
     * calls createConstraintsForPlan on each plan
     *
     * @param plans
     */
    @Override
    public void createConstraints(List<Plan> plans) {
        for (Plan plan : plans) {
            createConstraintsForPlan(plan);
        }

    }

    @Override
    public void createConstraintsForPlan(Plan plan) {
        String destinationPathWithoutName = cutDestinationPathToDirectory(plan);
        String constraintHeaderPath = Paths.get(generatedSourcesManager.getIncludeDir(),
                destinationPathWithoutName, "constraints").toString();
        File cstrIncPathOnDisk = new File(constraintHeaderPath);
        if (cstrIncPathOnDisk.exists() == false) {
            cstrIncPathOnDisk.mkdir();
        }
        String headerPath = Paths.get(constraintHeaderPath, plan.getName() + plan.getId() + "Constraints.h").toString();
        String fileContentHeader = xtendTemplates.constraintsHeader(plan);
        writeSourceFile(headerPath, fileContentHeader);

        formatFile(headerPath);

        String constraintSourcePath = Paths.get(generatedSourcesManager.getSrcDir(), destinationPathWithoutName, "constraints").toString();
        File cstrSrcPathOnDisk = new File(constraintSourcePath);
        if (cstrSrcPathOnDisk.exists() == false) {
            cstrSrcPathOnDisk.mkdir();
        }

        String srcPath = Paths.get(constraintSourcePath, plan.getName() + plan.getId() + "Constraints.cpp").toString();
        String fileContentSource = xtendTemplates.constraintsSource(plan, getActiveConstraintCodeGenerator(), packageName);
        writeSourceFile(srcPath, fileContentSource);
        formatFile(srcPath);

        for (State inPlan : plan.getStates()) {
            try {
                LineNumberReader lineNumberReader = new LineNumberReader(new FileReader(srcPath));
                while (lineNumberReader.ready()) {
                    if (lineNumberReader.readLine().contains("// State: " + inPlan.getName())) {
                        generatedSourcesManager.putLineForModelElement(inPlan.getId(), lineNumberReader.getLineNumber());
                        break;
                    }
                }
                lineNumberReader.close();
            } catch (IOException e) {
                LOG.error("Could not open/read lines for " + srcPath, e);
            }
        }
    }

    @Override
    public void createConstraintsForBehaviour(Behaviour behaviour) {
        String destinationPathWithoutName = cutDestinationPathToDirectory(behaviour);
        String constraintHeaderPath = Paths.get(generatedSourcesManager.getIncludeDir(),
                destinationPathWithoutName, "constraints").toString();
        File cstrIncPathOnDisk = new File(constraintHeaderPath);
        if (cstrIncPathOnDisk.exists() == false) {
            cstrIncPathOnDisk.mkdir();
        }
        String headerPath = Paths.get(constraintHeaderPath, behaviour.getName() + behaviour.getId() + "Constraints.h").toString();
        String fileContentHeader = xtendTemplates.constraintsHeader(behaviour);
        writeSourceFile(headerPath, fileContentHeader);

        formatFile(headerPath);

        String constraintSourcePath = Paths.get(generatedSourcesManager.getSrcDir(), destinationPathWithoutName, "constraints").toString();
        File cstrSrcPathOnDisk = new File(constraintSourcePath);
        if (cstrSrcPathOnDisk.exists() == false) {
            cstrSrcPathOnDisk.mkdir();
        }

        String srcPath = Paths.get(constraintSourcePath, behaviour.getName() + behaviour.getId() + "Constraints.cpp").toString();
        String fileContentSource = xtendTemplates.constraintsSource(behaviour, getActiveConstraintCodeGenerator(), packageName);
        writeSourceFile(srcPath, fileContentSource);
        formatFile(srcPath);
    }

    /**
     * calls createPlan for each plan
     *
     * @param plans list of all plans to generate (usually this should be all plans in workspace)
     */
    @Override
    public void createPlans(List<Plan> plans) {
        for (Plan plan : plans) {
            createPlan(plan);
        }
    }

    @Override
    public void createPlan(Plan plan) {
        String destinationPath = cutDestinationPathToDirectory(plan);

        String headerPath = Paths.get(generatedSourcesManager.getIncludeDir(), destinationPath, plan.getName() + plan.getId() + ".h").toString();
        String fileContentHeader = xtendTemplates.planHeader(plan, packageName);
        writeSourceFile(headerPath, fileContentHeader);

        formatFile(headerPath);

        String srcPath = Paths.get(generatedSourcesManager.getSrcDir(), destinationPath, plan.getName() + plan.getId() + ".cpp").toString();
        String fileContentSource = xtendTemplates.planSource(plan, getActiveConstraintCodeGenerator(), packageName);
        writeSourceFile(srcPath, fileContentSource);

        formatFile(srcPath);

        RuntimeCondition runtimeCondition = plan.getRuntimeCondition();
        if (runtimeCondition != null) {
            try {
                LineNumberReader lineNumberReader = new LineNumberReader(new FileReader(srcPath));
                while (lineNumberReader.ready()) {
                    if (lineNumberReader.readLine().contains("/*PROTECTED REGION ID(" + runtimeCondition.getId() + ") ENABLED START*/")) {
                        generatedSourcesManager.putLineForModelElement(runtimeCondition.getId(), lineNumberReader.getLineNumber());
                        break;
                    }

                }
                lineNumberReader.close();
            } catch (IOException e) {
                LOG.error("Could not open/read lines for " + srcPath, e);
            }
        }

        PreCondition preCondition = plan.getPreCondition();
        if (preCondition != null) {
            try {
                LineNumberReader lineNumberReader = new LineNumberReader(new FileReader(srcPath));
                while (lineNumberReader.ready()) {
                    if (lineNumberReader.readLine().contains("/*PROTECTED REGION ID(" + preCondition.getId() + ") ENABLED START*/")) {
                        generatedSourcesManager.putLineForModelElement(preCondition.getId(), lineNumberReader.getLineNumber());
                        break;
                    }

                }
                lineNumberReader.close();
            } catch (IOException e) {
                LOG.error("Could not open/read lines for " + srcPath, e);
            }
        }

        // TODO: Remove generation of transitions eval functions in plans when in alica_tests,
        // supplementary_tests, alica_turtle_sim and other packages have been updated to use the new transitions
        for (Transition inPlan : plan.getTransitions()) {
            try {
                LineNumberReader lineNumberReader = new LineNumberReader(new FileReader(srcPath));
                while (lineNumberReader.ready()) {
                    if (lineNumberReader.readLine().contains("/*PROTECTED REGION ID(" + inPlan.getId() + ") ENABLED START*/")) {
                        generatedSourcesManager.putLineForModelElement(inPlan.getId(), lineNumberReader.getLineNumber());
                        break;
                    }

                }
                lineNumberReader.close();
            } catch (IOException e) {
                LOG.error("Could not open/read lines for " + srcPath, e);
            }
        }
    }

    private String cutDestinationPathToDirectory(AbstractPlan plan) {
        String destinationPath = plan.getRelativeDirectory();
        if (destinationPath != null && destinationPath.lastIndexOf('.') > destinationPath.lastIndexOf(File.separator)) {
            destinationPath = destinationPath.substring(0, destinationPath.lastIndexOf(File.separator) + 1);
        }
        if (destinationPath == null) {
            return "";
        } else {
            return destinationPath;
        }
    }

    //TODO: remove duplicated code
    private String cutDestinationPathToDirectory(Condition condition) {
        String destinationPath = condition.getRelativeDirectory();
        if (destinationPath != null && destinationPath.lastIndexOf('.') > destinationPath.lastIndexOf(File.separator)) {
            destinationPath = destinationPath.substring(0, destinationPath.lastIndexOf(File.separator) + 1);
        }
        if (destinationPath == null) {
            return "";
        } else {
            return destinationPath;
        }
    }

    @Override
    public void createUtilityFunctionCreator(List<Plan> plans) {
        String headerPath = Paths.get(generatedSourcesManager.getIncludeDir(), "UtilityFunctionCreator.h").toString();
        String fileContentHeader = xtendTemplates.utilityFunctionCreatorHeader();
        writeSourceFile(headerPath, fileContentHeader);

        formatFile(headerPath);

        String srcPath = Paths.get(generatedSourcesManager.getSrcDir(), "UtilityFunctionCreator.cpp").toString();
        String fileContentSource = xtendTemplates.utilityFunctionCreatorSource(plans, packageName);
        writeSourceFile(srcPath, fileContentSource);

        formatFile(srcPath);
    }

    @Override
    public void createDomainCondition() {
        String headerPath = Paths.get(generatedSourcesManager.getIncludeDir(), "DomainCondition.h").toString();
        String fileContentHeader = xtendTemplates.domainConditionHeader();
        writeSourceFile(headerPath, fileContentHeader);

        formatFile(headerPath);

        String srcPath = Paths.get(generatedSourcesManager.getSrcDir(), "DomainCondition.cpp").toString();
        String fileContentSource = xtendTemplates.domainConditionSource(packageName);
        writeSourceFile(srcPath, fileContentSource);

        formatFile(srcPath);
    }

    @Override
    public void createDomainBehaviour() {
        String headerPath = Paths.get(generatedSourcesManager.getIncludeDir(), "DomainBehaviour.h").toString();
        String fileContentHeader = xtendTemplates.domainBehaviourHeader();
        writeSourceFile(headerPath, fileContentHeader);

        formatFile(headerPath);

        String srcPath = Paths.get(generatedSourcesManager.getSrcDir(), "DomainBehaviour.cpp").toString();
        String fileContentSource = xtendTemplates.domainBehaviourSource(packageName);
        writeSourceFile(srcPath, fileContentSource);

        formatFile(srcPath);
    }

    @Override
    public void createDomainPlan() {
        String headerPath = Paths.get(generatedSourcesManager.getIncludeDir(), "DomainPlan.h").toString();
        String fileContentHeader = xtendTemplates.domainPlanHeader();
        writeSourceFile(headerPath, fileContentHeader);

        formatFile(headerPath);

        String srcPath = Paths.get(generatedSourcesManager.getSrcDir(), "DomainPlan.cpp").toString();
        String fileContentSource = xtendTemplates.domainPlanSource(packageName);
        writeSourceFile(srcPath, fileContentSource);

        formatFile(srcPath);
    }

    @Override
    public void createTransitionConditions(List<Condition> conditions) {
        String destinationPath = cutDestinationPathToDirectory(conditions.get(0));
        String headerPath = Paths.get(generatedSourcesManager.getIncludeDir(), destinationPath, "conditions.h").toString();
        String fileContentHeader = xtendTemplates.transitionConditionHeader(conditions);
        writeSourceFile(headerPath, fileContentHeader);
        formatFile(headerPath);

        String srcPath = Paths.get(generatedSourcesManager.getSrcDir(), destinationPath, "conditions.cpp").toString();
        String fileContentSource = xtendTemplates.transitionConditionSource(conditions, packageName);
        writeSourceFile(srcPath, fileContentSource);
        formatFile(srcPath);
    }

    @Override
    public void createTransitionConditionsCreator(List<Condition> conditions) {
        String headerPath = Paths.get(generatedSourcesManager.getIncludeDir(), "TransitionConditionCreator.h").toString();
        String fileContentHeader = xtendTemplates.transitionConditionCreatorHeader(conditions);
        writeSourceFile(headerPath, fileContentHeader);
        formatFile(headerPath);

        String srcPath = Paths.get(generatedSourcesManager.getSrcDir(), "TransitionConditionCreator.cpp").toString();
        String fileContentSource = xtendTemplates.transitionConditionCreatorSource(conditions);
        writeSourceFile(srcPath, fileContentSource);
        formatFile(srcPath);
    }

    @Override
    public void setFormatter(String formatter) {
        this.formatter = formatter;
    }

    /**
     * This returns the {@link IConstraintCodeGenerator} of the active newCondition plugin.
     * TODO This maybe a candidate for a default method.
     *
     * @return
     */
    @Override
    public IConstraintCodeGenerator getActiveConstraintCodeGenerator() {
        return PluginManager.getInstance().getDefaultPlugin().getConstraintCodeGenerator();
    }


    private <T> void useTemplateAndSaveResults(String sourcePath, String headerPath, T objectToInteractWith,
                                               Function<T, String> templateForHeader, Function<T, String> templateForSource) {
        String fileContentHeader = templateForHeader.apply(objectToInteractWith);
        writeSourceFile(headerPath, fileContentHeader);
        formatFile(headerPath);


        String fileContentSource = templateForSource.apply(objectToInteractWith);
        writeSourceFile(sourcePath, fileContentSource);
        formatFile(sourcePath);
    }

    /**
     * Calls the executable found by the formatter attribute on the file found by filename.
     * It is assumed that the executable is clang-format or has the same CLI as clang-format.
     *
     * @param fileName
     */
    private void formatFile(String fileName) {
        if (formatter != null && formatter.length() > 0) {
            // clang-format will look for a file called .clang-format in the directory of the target file.
            // If it doesn't find any it will cd .. and try again.
            // https://stackoverflow.com/a/46374122
            String clangFormatName = ".clang-format";
            URL clangFormatUrl = CPPGeneratorImpl.class.getResource(clangFormatName);
//            System.out.println(clangFormatUrl.toString());
            File clangFormatFile = new File(clangFormatUrl.getFile());

            // copy .clang-format to file dir if not exists
//            String clangFormatDstStr = fileName.substring(0, fileName.lastIndexOf(File.separator)) + File.separator + clangFormatName;
//            File clangFormatDstFile = new File(clangFormatDstStr);
//            if (!clangFormatDstFile.exists()) {
//                try {
//                    Files.copy(clangFormatFile.toPath(), clangFormatDstFile.toPath());
//                } catch (IOException e) {
//                    // intercept a bug, by move a File
//                    if(!(e instanceof FileAlreadyExistsException)) {
//                        LOG.error("An error occurred while copying format style to destination", e);
//                        throw new RuntimeException(e);
//                    }
//                }
//            }

            // run formatter
            String command = formatter + " --style=file -i " + fileName;
            try {
                Runtime.getRuntime().exec(command).waitFor();
            } catch (IOException | InterruptedException e) {
                LOG.error("An error occurred while formatting generated sources", e);
                throw new RuntimeException(e);
            }

            // remove .clang-format from file dir
//            clangFormatDstFile.delete();
        } else {
            LOG.warn("Generated files are not formatted because no formatter is configured");
        }
    }
}
