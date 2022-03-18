package de.unikassel.vs.alica.planDesigner.command.add;

import de.unikassel.vs.alica.planDesigner.alicamodel.EntryPoint;
import de.unikassel.vs.alica.planDesigner.alicamodel.Task;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;

public class AddTaskToEntryPoint extends Command {
    protected EntryPoint entryPoint;
    protected Task newTask;
    protected Task oldTask;

    public AddTaskToEntryPoint(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.entryPoint = (EntryPoint) modelManager.getPlanElement(mmq.getParentId());
        this.oldTask = this.entryPoint.getTask();
        this.newTask = (Task) modelManager.getPlanElement(mmq.getElementId());
    }

    @Override
    public void doCommand() {
        entryPoint.setTask(newTask);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED, oldTask);
        this.fireEvent(ModelEventType.ELEMENT_ADDED, newTask);
    }

    @Override
    public void undoCommand() {
        this.entryPoint.setTask(oldTask);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED, newTask);
        this.fireEvent(ModelEventType.ELEMENT_ADDED, oldTask);}
}
