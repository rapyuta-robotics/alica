package de.unikassel.vs.alica.generator;

import de.unikassel.vs.alica.planDesigner.alicamodel.*;

import java.io.File;
import java.io.IOException;
import java.util.*;
import java.nio.file.Paths;

public class GeneratedSourcesManager {

    private String codegenPath;
    private String editorExecutablePath;
    private Map<Long, Integer> linesForGeneratedElements;

    public GeneratedSourcesManager() {
        linesForGeneratedElements = new HashMap<>();
    }

    // Code generation path' getter and setter
    public void setCodegenPath(String codegenPath) {
        this.codegenPath = codegenPath;
    }
    public String getCodegenPath() {
        return codegenPath;
    }
    public String getIncludeDir() {
        return Paths.get(codegenPath, "include").toString();
    }
    public String getSrcDir() {
        return Paths.get(codegenPath, "src").toString();
    }

    public void setEditorExecutablePath(String editorExecutablePath) {
        this.editorExecutablePath = editorExecutablePath;
    }

    public void showSourceCode(PlanElement planElement) {
        List<File> allGeneratedFilesForPlanElement = new ArrayList<>();
        int lineNumber = 0;
        if (planElement instanceof Behaviour) {
            allGeneratedFilesForPlanElement.addAll(getGeneratedFilesForBehaviour((Behaviour) planElement));
        } else if (planElement instanceof State) {
            allGeneratedFilesForPlanElement.addAll(getGeneratedConditionFilesForPlan(((State) planElement).getParentPlan()));
        } else if (planElement instanceof Transition) {
            allGeneratedFilesForPlanElement.addAll(getGeneratedConditionFilesForPlan(((Transition) planElement).getOutState().getParentPlan()));
        } else if (planElement instanceof Plan) {
            allGeneratedFilesForPlanElement.addAll(getGeneratedConditionFilesForPlan((Plan) planElement));
            allGeneratedFilesForPlanElement.addAll(getGeneratedConstraintFilesForPlan((Plan) planElement));
        } else {
            throw new RuntimeException("File for unkown generated source elementType requested!");
        }
        lineNumber = getLineNumberForGeneratedElement(planElement.getId());

        for (File generatedSource : allGeneratedFilesForPlanElement) {
            try {
                Runtime.getRuntime().exec(editorExecutablePath + " " + generatedSource.getAbsolutePath() + " +" + lineNumber);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public List<File> getGeneratedFilesForBehaviour(Behaviour behaviour) {
        List<File> generatedFiles = new ArrayList<>();
        String destinationPath = trimFileFromPath(behaviour.getRelativeDirectory());
        generatedFiles.add(Paths.get(getIncludeDir(), destinationPath, behaviour.getName() + ".h").toFile());
        generatedFiles.add(Paths.get(getSrcDir(), destinationPath, behaviour.getName() + ".cpp").toFile());
        generatedFiles.add(Paths.get(getIncludeDir(), destinationPath, behaviour.getName() + behaviour.getId() + ".h").toFile());
        generatedFiles.add(Paths.get(getSrcDir(), destinationPath, behaviour.getName() + behaviour.getId() + ".cpp").toFile());
        return generatedFiles;
    }

    public List<File> getGeneratedConditionFilesForPlan(AbstractPlan abstractPlan) {
        List<File> generatedFiles = new ArrayList<>();
        String destinationPath = trimFileFromPath(abstractPlan.getRelativeDirectory());
        String headerFilename = abstractPlan.getName() + abstractPlan.getId() + ".h";
        String sourceFilename = abstractPlan.getName() + abstractPlan.getId() + ".cpp";
        generatedFiles.add(Paths.get(getIncludeDir(), destinationPath, headerFilename).toFile());
        generatedFiles.add(Paths.get(getSrcDir(), destinationPath, sourceFilename).toFile());
        return generatedFiles;
    }

    public List<File> getGeneratedConstraintFilesForPlan(AbstractPlan abstractPlan) {
        List<File> generatedFiles = new ArrayList<>();
        String destinationPath = trimFileFromPath(abstractPlan.getRelativeDirectory());
        String constraintHeaderFileName = abstractPlan.getName() + abstractPlan.getId() + "Constraints.h";
        String constraintSourceFileName = abstractPlan.getName() + abstractPlan.getId() + "Constraints.cpp";
        generatedFiles.add(Paths.get(getIncludeDir(), destinationPath, "constraints", constraintHeaderFileName).toFile());
        generatedFiles.add(Paths.get(getSrcDir(), destinationPath, "constraints", constraintSourceFileName).toFile());
        return generatedFiles;
    }

    public List<File> getGeneratedConstraintFilesForBehaviour(Behaviour behaviour) {
        List<File> generatedFiles = new ArrayList<>();
        String destinationPath = trimFileFromPath(behaviour.getRelativeDirectory());
        String constraintHeaderFileName = behaviour.getName() + behaviour.getId() + "Constraints.h";
        String constraintSourceFileName = behaviour.getName() + behaviour.getId() + "Constraints.cpp";
        generatedFiles.add(Paths.get(getIncludeDir(), destinationPath, "constraints", constraintHeaderFileName).toFile());
        generatedFiles.add(Paths.get(getSrcDir(), destinationPath, "constraints", constraintSourceFileName).toFile());
        return generatedFiles;
    }

    private String trimFileFromPath(String destinationPath) {
        if (destinationPath.lastIndexOf('.') > destinationPath.lastIndexOf(File.separator)) {
            return destinationPath.substring(0, destinationPath.lastIndexOf(File.separator) + 1);
        } else {
            return destinationPath;
        }
    }

    /**
     * delegate {@link Map#put(Object, Object)}
     *
     * @param modelElementId
     * @param lineNumber
     */
    public void putLineForModelElement(long modelElementId, Integer lineNumber) {
        linesForGeneratedElements.put(modelElementId, lineNumber);
    }

    /**
     * if code has not been generated this method returns 0
     *
     * @param modelElementId
     * @return line number of generated code
     */
    public int getLineNumberForGeneratedElement(long modelElementId) {
        if (linesForGeneratedElements.containsKey(modelElementId)) {
            return linesForGeneratedElements.get(modelElementId);
        } else {
            return 0;
        }
    }
}
