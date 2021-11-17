package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import de.unikassel.vs.alica.planDesigner.alicamodel.AbstractPlan;
import de.unikassel.vs.alica.planDesigner.alicamodel.ConfAbstractPlanWrapper;
import de.unikassel.vs.alica.planDesigner.alicamodel.Configuration;
import javafx.util.Pair;
import org.w3c.dom.Element;

public class ConfAbstractPlanWrapperFactory {

    public static ConfAbstractPlanWrapper create(Element abstractPlanNode, ConversionProcess cp) {
        // Note: The wrapper did not exist before, so it is okay to let it keep its own new ID, empty name, and comment.
        ConfAbstractPlanWrapper confAbstractPlanWrapper = new ConfAbstractPlanWrapper();
        cp.addElement(confAbstractPlanWrapper);
        Long abstractPlanID = cp.getReferencedId(abstractPlanNode.getTextContent());
        if (abstractPlanID != -1) {
            cp.wrapperAbstractPlanReferences.put(confAbstractPlanWrapper.getId(), abstractPlanID);
        }
        return confAbstractPlanWrapper;
    }

    public static void attachReferences(ConversionProcess cp) {
        for (Pair<Long, Long> entry : cp.wrapperAbstractPlanReferences.getEntries()) {
            ConfAbstractPlanWrapper confAbstractPlanWrapper = (ConfAbstractPlanWrapper) cp.getElement(entry.getKey());
            if (confAbstractPlanWrapper == null) {
                throw new RuntimeException("[StateFactory] ConfAbstractPlanWrapper with ID " + entry.getKey() + " unknown!");
            }
            AbstractPlan abstractPlan;
            if (cp.configurationBehaviourMapping.containsKey(entry.getValue())) {
                abstractPlan = (AbstractPlan) cp.getElement(cp.configurationBehaviourMapping.get(entry.getValue()));

                /**
                 * Note: This section is really special, so this note should help to understand
                 * what is happening here!
                 *
                 * Configurations don't have the relativeDirectory property set, yet, because this is normally set
                 * with the help of the absolute path to their file. Configurations are created on the fly, when
                 * behaviours are created and therefore configurations don't have a file, yet. Our solution is
                 * to serialise configurations to the same place as their corresponding behaviour as serialised to.
                 *
                 * That is why we set the relativeDirectory of configurations to that of behaviours from the
                 * configurationBehaviourMapping.
                 */
                Configuration configuration = (Configuration) cp.getElement(entry.getValue());
                if (configuration != null) {
                    configuration.setRelativeDirectory(abstractPlan.getRelativeDirectory());
                    confAbstractPlanWrapper.setConfiguration(configuration);
                }
            } else {
                abstractPlan = (AbstractPlan) cp.getElement(entry.getValue());
            }
            if (abstractPlan == null) {
                throw new RuntimeException("[ConfAbstractPlanWrapperFactory] Abstract plan with ID " + entry.getValue() + " unknown!");
            }
            confAbstractPlanWrapper.setAbstractPlan(abstractPlan);
        }
        cp.wrapperAbstractPlanReferences.clear();
    }
}
