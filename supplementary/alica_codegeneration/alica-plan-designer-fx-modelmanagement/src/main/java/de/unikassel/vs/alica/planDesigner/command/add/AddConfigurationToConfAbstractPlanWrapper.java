package de.unikassel.vs.alica.planDesigner.command.add;

import de.unikassel.vs.alica.planDesigner.alicamodel.ConfAbstractPlanWrapper;
import de.unikassel.vs.alica.planDesigner.alicamodel.Configuration;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;

public class AddConfigurationToConfAbstractPlanWrapper extends Command {

        protected ConfAbstractPlanWrapper wrapper;
        protected Configuration configuration;
        protected Configuration oldConfiguration;

        public AddConfigurationToConfAbstractPlanWrapper(ModelManager modelManager, ModelModificationQuery mmq) {
            super(modelManager, mmq);
            wrapper = (ConfAbstractPlanWrapper) modelManager.getPlanElement(mmq.getParentId());
            oldConfiguration = wrapper.getConfiguration();
            configuration = (Configuration) modelManager.getPlanElement(mmq.getElementId());
        }

        @Override
        public void doCommand() {
            wrapper.setConfiguration(configuration);
            fireEvent(ModelEventType.ELEMENT_ADDED, configuration);
        }

        @Override
        public void undoCommand() {
            wrapper.setConfiguration(oldConfiguration);
            fireEvent(ModelEventType.ELEMENT_REMOVED, configuration);
        }
}
