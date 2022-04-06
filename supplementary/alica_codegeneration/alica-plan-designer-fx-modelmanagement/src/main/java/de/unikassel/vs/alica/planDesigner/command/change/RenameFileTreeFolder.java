package de.unikassel.vs.alica.planDesigner.command.change;

import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.command.ChangeAttributeCommand;
import de.unikassel.vs.alica.planDesigner.events.ModelEvent;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.FileSystemUtil;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class RenameFileTreeFolder extends ChangeAttributeCommand {
    private Object newValue;
    private Object oldValue;
    private String elementType;
    private String ending;
    private Boolean emptyFolder = true;

    public RenameFileTreeFolder(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.oldValue = mmq.getAttributeName();
        this.newValue = mmq.getNewValue();
    }
    @Override
    public void doCommand() {
        emptyFolder = true;
        for (PlanElement planElement: this.modelManager.getPlanElements()) {
            // TODO Not implement for RoleSet, RoleSet hasn't AbsoluteDirectory
            if(planElement instanceof Plan || planElement instanceof Behaviour || planElement instanceof  TaskRepository
                    || planElement instanceof PlanType) {
                if (modelManager.getAbsoluteDirectory(planElement).contains(oldValue.toString())) {
                    try {
                        Files.createDirectories(Paths.get(newValue.toString()));
                        if(planElement instanceof Plan) { elementType = Types.PLAN; }
                        if(planElement instanceof PlanType) { elementType = Types.PLANTYPE; }
                        if(planElement instanceof Behaviour) { elementType = Types.BEHAVIOUR; }
                        if(planElement instanceof TaskRepository) { elementType = Types.TASKREPOSITORY; }
                        ending = FileSystemUtil.getType((SerializablePlanElement) planElement);
                        this.modelManager.moveFile(((SerializablePlanElement) planElement),elementType, newValue.toString(), ending);
                        fireEvent(((SerializablePlanElement) planElement), elementType, "relativeDirectory");
                        emptyFolder = false;
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
        if(emptyFolder) {
            try {
                Files.createDirectories(Paths.get(newValue.toString()));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        File file = new File(oldValue.toString());
        file.delete();
        ModelEvent modelEvent = new ModelEvent(ModelEventType.FOLDER_DELETED, null, null);
        modelEvent.setChangedAttribute(oldValue.toString());
        this.modelManager.fireEvent(modelEvent);
    }
    @Override
    public void undoCommand() {
        emptyFolder = true;
        for (PlanElement planElement: this.modelManager.getPlanElements()) {
            // TODO Not implement for RoleSet, RoleSet hasn't AbsoluteDirectory
            if(planElement instanceof Plan || planElement instanceof Behaviour || planElement instanceof  TaskRepository
                    || planElement instanceof PlanType) {
                if (modelManager.getAbsoluteDirectory(planElement).contains(newValue.toString())) {
                    try {
                        Files.createDirectories(Paths.get(oldValue.toString()));
                        if(planElement instanceof Plan) { elementType = Types.PLAN; }
                        if(planElement instanceof PlanType) { elementType = Types.PLANTYPE; }
                        if(planElement instanceof Behaviour) { elementType = Types.BEHAVIOUR; }
                        if(planElement instanceof TaskRepository) { elementType = Types.TASKREPOSITORY; }
                        ending = FileSystemUtil.getType((SerializablePlanElement) planElement);
                        this.modelManager.moveFile(((SerializablePlanElement) planElement),elementType, oldValue.toString(), ending);
                        fireEvent(((SerializablePlanElement) planElement), elementType, "relativeDirectory");
                        emptyFolder = false;
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
        if(emptyFolder) {
            try {
                Files.createDirectories(Paths.get(oldValue.toString()));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        File file = new File(newValue.toString());
        file.delete();
        ModelEvent modelEvent = new ModelEvent(ModelEventType.FOLDER_DELETED, null, null);
        modelEvent.setChangedAttribute(newValue.toString());
        this.modelManager.fireEvent(modelEvent);
    }
}
