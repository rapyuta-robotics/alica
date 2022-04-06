package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.Configuration;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CreateConfiguration extends Command {
    Configuration configuration;

    public CreateConfiguration(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.configuration = createConfiguration();
    }

    protected Configuration createConfiguration() {
        Configuration configuration = new Configuration();
        configuration.setName(mmq.getName());
        configuration.setRelativeDirectory(modelManager.makeRelativeDirectory(mmq.getAbsoluteDirectory(), configuration.getName()+ "." + Extensions.CONFIGURATION));
        configuration.registerDirtyFlag();
        return configuration;
    }

    @Override
    public void doCommand() {
        modelManager.storePlanElement(Types.CONFIGURATION, this.configuration,true);
        this.fireEvent(ModelEventType.ELEMENT_CREATED, this.configuration);
    }

    @Override
    public void undoCommand() {
        modelManager.dropPlanElement(Types.CONFIGURATION, this.configuration, true);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, this.configuration);
    }
}
