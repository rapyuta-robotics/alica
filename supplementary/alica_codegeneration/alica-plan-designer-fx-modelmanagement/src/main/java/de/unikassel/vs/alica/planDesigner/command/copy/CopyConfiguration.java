package de.unikassel.vs.alica.planDesigner.command.copy;

import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Extensions;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

import java.util.Map;

public class CopyConfiguration extends Command {

    private Configuration configuration;
    private Configuration copyConfiguration = new Configuration();

    public CopyConfiguration(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.configuration = (Configuration) modelManager.getPlanElement(mmq.getElementId());
    }

    @Override
    public void doCommand() {
        copyConfiguration.setName(configuration.getName() + "Copy");
        copyConfiguration.setComment(configuration.getComment());
        copyConfiguration.setRelativeDirectory(modelManager.makeRelativeDirectory(configuration.getRelativeDirectory(), copyConfiguration.getName()+ "." + Extensions.CONFIGURATION));
        //set parameters
        for (Map.Entry<String, String> entry : configuration.getParameters().entrySet()) {
            copyConfiguration.putParameter(entry.getKey(), entry.getValue());
        }

        this.modelManager.storePlanElement(Types.CONFIGURATION, copyConfiguration,true);
        this.fireEvent(ModelEventType.ELEMENT_CREATED, copyConfiguration);
    }

    @Override
    public void undoCommand() {
        modelManager.dropPlanElement(Types.CONFIGURATION, copyConfiguration, true);
        this.fireEvent(ModelEventType.ELEMENT_DELETED, copyConfiguration);
    }
}
