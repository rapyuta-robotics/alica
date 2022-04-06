package de.unikassel.vs.alica.planDesigner.command;

import com.google.common.io.Files;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;
import de.unikassel.vs.alica.planDesigner.events.ModelEvent;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.events.ModelQueryType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.FileSystemUtil;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;

public class MoveFolder extends ChangeAttributeCommand {
    private File oldFile;
    private String newAbsoluteDirectory;
    private File[] allFiles;
    private String ending;
    private String types;
    MoveFolder moveFolder;


    public MoveFolder(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.oldFile = new File(mmq.getName()) ;
        this.newAbsoluteDirectory = mmq.getAbsoluteDirectory();
        this.allFiles = oldFile.listFiles();
    }
    @Override
    public void doCommand() {
        File newFile = new File(newAbsoluteDirectory + "/" + oldFile.getName());

        //if folder is empty
        if(allFiles.length == 0) {
            try {
                Files.move(oldFile,newFile);
            } catch (IOException e) {
                e.printStackTrace();
            }

            ModelEvent modelEvent = new ModelEvent(ModelEventType.FOLDER_DELETED, null, null);
            modelEvent.setChangedAttribute(oldFile.toString());
            this.modelManager.fireEvent(modelEvent);
        } else {
            //create a new File
            try {
                java.nio.file.Files.createDirectories(Paths.get(newFile.toString()));
            } catch (IOException e) {
                e.printStackTrace();
            }

            for (File file: allFiles) {
                PlanElement planElement = modelManager.getPlanElement(file.getPath());
                if (planElement != null) {
                    if (planElement instanceof Behaviour) {
                        types = Types.BEHAVIOUR;
                    }
                    if (planElement instanceof Plan) {
                        types = Types.PLAN;
                    }
                    if (planElement instanceof PlanType) {
                        types = Types.PLANTYPE;
                    }
                    this.ending = FileSystemUtil.getType((SerializablePlanElement) planElement);
                    this.modelManager.moveFile((SerializablePlanElement) planElement, types, newFile.getAbsolutePath(), ending);
                    fireEvent((SerializablePlanElement) planElement, types, "relativeDirectory");
                } else {
                    //recursive call, if subFolders exists
                    String name = oldFile + "/" + file.getName();
                    String directory = newFile.toString() ;
                    ModelModificationQuery newMMQ = new ModelModificationQuery(ModelQueryType.MOVE_FILE, directory, Types.FOLDER, name);
                    moveFolder = new MoveFolder(modelManager, newMMQ);
                    moveFolder.doCommand();
                }
            }
            oldFile.delete();
            ModelEvent modelEvent = new ModelEvent(ModelEventType.FOLDER_DELETED, null, null);
            modelEvent.setChangedAttribute(oldFile.toString());
            this.modelManager.fireEvent(modelEvent);
        }
    }

    @Override
    public void undoCommand() {
        File newFile = new File(newAbsoluteDirectory + "/" + oldFile.getName());

        //if folder is empty
        if(allFiles.length == 0) {
            try {
                Files.move(newFile, oldFile);
            } catch (IOException e) {
                e.printStackTrace();
            }
            ModelEvent modelEvent = new ModelEvent(ModelEventType.FOLDER_DELETED, null, null);
            modelEvent.setChangedAttribute(newFile.toString());
            this.modelManager.fireEvent(modelEvent);
        } else {
            //create a new File
            try {
                java.nio.file.Files.createDirectories(Paths.get(oldFile.toString()));
            } catch (IOException e) {
                e.printStackTrace();
            }

            for (File file: allFiles) {
                PlanElement planElement = modelManager.getPlanElement(file.getPath());
                if (planElement != null) {
                    if (planElement instanceof Behaviour) {
                        types = Types.BEHAVIOUR;
                    }
                    if (planElement instanceof Plan) {
                        types = Types.PLAN;
                    }
                    if (planElement instanceof PlanType) {
                        types = Types.PLANTYPE;
                    }
                    this.ending = FileSystemUtil.getType((SerializablePlanElement) planElement);
                    this.modelManager.moveFile((SerializablePlanElement) planElement, types, oldFile.getAbsolutePath(), ending);
                    fireEvent((SerializablePlanElement) planElement, types, "relativeDirectory");
                } else {
                    moveFolder.undoCommand();
                }
            }
            newFile.delete();
            ModelEvent modelEvent = new ModelEvent(ModelEventType.FOLDER_DELETED, null, null);
            modelEvent.setChangedAttribute(newFile.toString());
            this.modelManager.fireEvent(modelEvent);
        }
    }
}
