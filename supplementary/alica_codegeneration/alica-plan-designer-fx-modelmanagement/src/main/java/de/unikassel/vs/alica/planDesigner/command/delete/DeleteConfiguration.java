package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.Configuration;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class DeleteConfiguration extends Command {

    protected Configuration configuration;

    public DeleteConfiguration(ModelManager manager, ModelModificationQuery mmq) {
        super(manager, mmq);
        configuration = (Configuration) manager.getPlanElement(mmq.getElementId());
        if (configuration == null) {
            throw new RuntimeException("DeleteConfiguration: Configuration with ID " + mmq.getElementId() + " is not known!");
        }
    }

    @Override
    public void doCommand() {
        modelManager.dropPlanElement(Types.CONFIGURATION, configuration, true);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, this.configuration);
    }

    @Override
    public void undoCommand() {
        modelManager.storePlanElement(Types.CONFIGURATION, configuration, true);
        this.fireEvent(ModelEventType.ELEMENT_CREATED, configuration);
    }
}
