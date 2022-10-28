package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.command.UiPositionCommand;
import de.unikassel.vs.alica.planDesigner.events.ModelEvent;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class DeleteFolder extends UiPositionCommand {

    public DeleteFolder(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
    }

    @Override
    public void doCommand() {
        File file = new File(mmq.getName());
        file.delete();
        ModelEvent modelEvent = new ModelEvent(ModelEventType.FOLDER_DELETED, null, null);
        modelEvent.setChangedAttribute(mmq.getName());
        this.modelManager.fireEvent(modelEvent);
    }
    @Override
    public void undoCommand() {
        try {
            Files.createDirectories(Paths.get(mmq.getName()));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
