package de.unikassel.vs.alica.generator;

import de.unikassel.vs.alica.generator.cpp.CPPGeneratorImpl;
import de.unikassel.vs.alica.generator.cpp.parser.CommentsLexer;
import de.unikassel.vs.alica.generator.cpp.parser.CommentsParser;
import de.unikassel.vs.alica.generator.cpp.parser.ProtectedRegionsVisitor;
import de.unikassel.vs.alica.generator.plugin.PluginManager;
import de.unikassel.vs.alica.planDesigner.alicamodel.AbstractPlan;
import de.unikassel.vs.alica.planDesigner.alicamodel.Behaviour;
import de.unikassel.vs.alica.planDesigner.alicamodel.Condition;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.List;

/**
 * General Code Generator. It manages calling the correct {@link IGenerator} implementation
 * and serves as a simple way of generating code for the rest of the application.
 * If you want to generate a file just call {@link Codegenerator#generate(AbstractPlan)}
 * or {@link Codegenerator#generate()} to generate all files.
 * <p>
 * Do not cache this object.
 * A new instance should be created for every use or at least after creating a new ALICA object.
 */
public class Codegenerator {

    private static final Logger LOG = LogManager.getLogger(Codegenerator.class);

    private final IGenerator languageSpecificGenerator;
    private final String codeGenerationDestination;
    private final GeneratedSourcesManager generatedSourcesManager;

    public StringProperty currentFile = new SimpleStringProperty();

    private List<Plan> plans;
    private List<Behaviour> behaviours;
    private List<Condition> conditions;

    /**
     * This constructor initializes a C++ code generator
     */
    public Codegenerator(List<Plan> plans, List<Behaviour> behaviours, List<Condition> conditions, String formatter, GeneratedSourcesManager generatedSourcesManager) {
        // TODO: Document this! Here can the programming language be changed.
        languageSpecificGenerator = new CPPGeneratorImpl(generatedSourcesManager);
        languageSpecificGenerator.setFormatter(formatter);
        codeGenerationDestination = generatedSourcesManager.getCodegenPath();
        this.generatedSourcesManager = generatedSourcesManager;

        this.plans = plans;
        Collections.sort(plans, new PlanElementComparator());
        this.behaviours = behaviours;
        Collections.sort(behaviours, new PlanElementComparator());
        this.conditions = conditions;
        Collections.sort(conditions, new PlanElementComparator());
    }

    /**
     * Generates source files for all ALICA plans and behaviours in workspace.
     */
    // TODO: To be reviewed and maybe adapted, because of MVC pattern adaption.
    public void generate() {
        ProtectedRegionsVisitor protectedRegionsVisitor = new ProtectedRegionsVisitor();
        try {
            if (Files.notExists(Paths.get(codeGenerationDestination))) {
                Files.createDirectories(Paths.get(codeGenerationDestination));
            }
            Files.walk(Paths.get(codeGenerationDestination)).filter(e -> {
                String fileName = e.getFileName().toString();
                return fileName.endsWith(".h") || fileName.endsWith(".cpp");
            }).forEach(e -> {
                try {
                    visit(protectedRegionsVisitor, e);
                    currentFile.set(e.toString());
                } catch (IOException e1) {
                    LOG.error("Could not parse existing source file " + e, e1);
                    throw new RuntimeException(e1);
                }
            });
        } catch (IOException e) {
            LOG.error("Could not find expression validator path! ", e);
            throw new RuntimeException(e);
        }

        PluginManager.getInstance().getDefaultPlugin().setProtectedRegions(protectedRegionsVisitor.getProtectedRegions());
        languageSpecificGenerator.setProtectedRegions(protectedRegionsVisitor.getProtectedRegions());

        languageSpecificGenerator.createDomainBehaviour();
        languageSpecificGenerator.createDomainCondition();
        languageSpecificGenerator.createDomainPlan();

        languageSpecificGenerator.createUtilityFunctionCreator(plans);
        languageSpecificGenerator.createBehaviourCreator(behaviours);
        languageSpecificGenerator.createConditionCreator(plans, behaviours, conditions);
        languageSpecificGenerator.createConstraintCreator(plans, behaviours, conditions);

        languageSpecificGenerator.createConstraints(plans);
        languageSpecificGenerator.createPlans(plans);
        languageSpecificGenerator.createPlanCreator(plans);

        for (Behaviour behaviour : behaviours) {
            languageSpecificGenerator.createBehaviour(behaviour);
            languageSpecificGenerator.createConstraintsForBehaviour(behaviour);
        }
        LOG.info("Generated all files successfully");
    }

    private void visit(ProtectedRegionsVisitor protectedRegionsVisitor, Path e) throws IOException {
        CommentsLexer lexer = new CommentsLexer(CharStreams.fromPath(e));
        CommonTokenStream commonTokenStream = new CommonTokenStream(lexer);
        CommentsParser parser = new CommentsParser(commonTokenStream);
        CommentsParser.All_textContext all_textContext = parser.all_text();
        protectedRegionsVisitor.visit(all_textContext);
    }

    /**
     * (Re)Generates source files for the given object.
     * If the given object is an instance of {@link Plan} or {@link Behaviour}.
     *
     * @param abstractPlan
     */
    public void generate(AbstractPlan abstractPlan) {
        if (abstractPlan instanceof Plan) {
            generate((Plan) abstractPlan);
        } else if (abstractPlan instanceof Behaviour) {
            generate((Behaviour) abstractPlan);
        } else {
            LOG.error("Nothing to generate for something else than a plan or behaviour!");
        }
    }

    public void generate(Plan plan) {
        List<File> generatedFiles = generatedSourcesManager.getGeneratedConditionFilesForPlan(plan);
        generatedFiles.addAll(generatedSourcesManager.getGeneratedConstraintFilesForPlan(plan));
        collectProtectedRegions(generatedFiles);
        languageSpecificGenerator.createConstraintsForPlan(plan);
        languageSpecificGenerator.createPlan(plan);
        languageSpecificGenerator.createConditionCreator(plans, behaviours, conditions);
        languageSpecificGenerator.createUtilityFunctionCreator(plans);
    }

    public void generate(Behaviour behaviour) {
        List<File> generatedFiles = generatedSourcesManager.getGeneratedFilesForBehaviour(behaviour);
        generatedFiles.addAll(generatedSourcesManager.getGeneratedConstraintFilesForBehaviour(behaviour));
        collectProtectedRegions(generatedFiles);
        languageSpecificGenerator.createBehaviourCreator(behaviours);
        languageSpecificGenerator.createConstraintsForBehaviour(behaviour);
        languageSpecificGenerator.createBehaviour(behaviour);
    }

    protected void collectProtectedRegions(List<File> filesToParse) {
        ProtectedRegionsVisitor protectedRegionsVisitor = new ProtectedRegionsVisitor();
        for (File genFile : filesToParse) {
            try {
                if (genFile.exists()) {
                    visit(protectedRegionsVisitor, genFile.toPath());
                }
            } catch (IOException e1) {
                LOG.error("Could not parse existing source file " + genFile.getAbsolutePath(), e1);
                throw new RuntimeException(e1);
            }
        }
        PluginManager.getInstance().getDefaultPlugin().setProtectedRegions(protectedRegionsVisitor.getProtectedRegions());
        languageSpecificGenerator.setProtectedRegions(protectedRegionsVisitor.getProtectedRegions());
    }
}
