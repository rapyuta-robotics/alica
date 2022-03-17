package de.unikassel.vs.alica.generator.plugin;

import de.unikassel.vs.alica.generator.IConstraintCodeGenerator;
import de.unikassel.vs.alica.generator.Codegenerator;
import de.unikassel.vs.alica.generator.IGenerator;
import de.unikassel.vs.alica.planDesigner.alicamodel.Condition;
import javafx.scene.Parent;

import java.io.File;
import java.io.IOException;
import java.util.Map;

/**
 * This interface defines the basic functionality that a plugin has to implement.
 */
public interface IPlugin<T> {

    /**
     * @return the custom {@link IConstraintCodeGenerator}.
     * This is the main functionality of the plugin from the perspective of the {@link Codegenerator}
     * or the {@link IGenerator}
     */
    IConstraintCodeGenerator getConstraintCodeGenerator();

    /**
     * Returns the plugin view that will be embedded into the properties view of the newCondition
     * @return
     */
    Parent getPluginUI() throws IOException;

    /**
     * Writes into the attributes of the given newCondition.
     * Do NOT write data into the hierarchy above the given newCondition.
     * @param condition
     */
    void writePluginValuesToCondition(Condition condition);

    /**
     * Reads necessary data for the plugin out of the newCondition.
     * @param condition
     * @return
     */
    T readPluginValuesFromCondition(Condition condition);

    /**
     * The name has to be a non-null string value.
     * It is assumed plugin names are unique.
     * @return the name of the plugin
     */
    String getName();

    /**
     * @return the JAR of the plugin
     */
    File getPluginFile();

    void setPluginFile(File pluginFile);

    /**
     * This method should make a delegate to the
     * {@link IConstraintCodeGenerator} and make the protected regions known for it.
     * It is mainly a reminder to not forget the implementation
     * @param protectedRegions
     */
    void setProtectedRegions(Map<String, String> protectedRegions);
}
