package de.unikassel.vs.alica.planDesigner.command.remove;

import de.unikassel.vs.alica.planDesigner.alicamodel.ConfAbstractPlanWrapper;
import de.unikassel.vs.alica.planDesigner.alicamodel.Configuration;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;

public class RemoveConfigurationFromWrapper extends Command {
    protected ConfAbstractPlanWrapper confAbstractPlanWrapper;
    protected Configuration configuration;

    public RemoveConfigurationFromWrapper(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.confAbstractPlanWrapper = (ConfAbstractPlanWrapper) modelManager.getPlanElement(mmq.getParentId());
        this.configuration = (Configuration) modelManager.getPlanElement(mmq.getElementId());
    }

    @Override
    public void doCommand() {
        this.confAbstractPlanWrapper.setConfiguration(null);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED, configuration);
    }

    @Override
    public void undoCommand() {
        this.confAbstractPlanWrapper.setConfiguration(configuration);
        this.fireEvent(ModelEventType.ELEMENT_ADDED, configuration);
    }
}
