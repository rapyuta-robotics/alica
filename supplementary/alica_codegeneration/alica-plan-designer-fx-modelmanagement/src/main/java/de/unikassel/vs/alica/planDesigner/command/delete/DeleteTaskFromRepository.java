package de.unikassel.vs.alica.planDesigner.command.delete;

import de.unikassel.vs.alica.planDesigner.alicamodel.Task;
import de.unikassel.vs.alica.planDesigner.alicamodel.TaskRepository;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class DeleteTaskFromRepository extends Command {

    protected Task task;
    protected TaskRepository taskRepository;

    public DeleteTaskFromRepository(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.task = (Task) modelManager.getPlanElement(mmq.getElementId());
        this.taskRepository = (TaskRepository) modelManager.getPlanElement(mmq.getParentId());
    }

    @Override
    public void doCommand() {
        this.taskRepository.removeTask(this.task);
        this.modelManager.dropPlanElement(Types.TASK, this.task, false);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, this.task);
    }

    @Override
    public void undoCommand() {
        this.taskRepository.addTask(this.task);
        this.modelManager.storePlanElement(Types.TASK, this.task, false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, this.task);
    }
}
