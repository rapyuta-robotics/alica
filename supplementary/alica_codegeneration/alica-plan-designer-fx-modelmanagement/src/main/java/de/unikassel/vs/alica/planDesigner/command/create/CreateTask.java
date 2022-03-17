package de.unikassel.vs.alica.planDesigner.command.create;

import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.alicamodel.SerializablePlanElement;
import de.unikassel.vs.alica.planDesigner.alicamodel.Task;
import de.unikassel.vs.alica.planDesigner.alicamodel.TaskRepository;
import de.unikassel.vs.alica.planDesigner.command.Command;
import de.unikassel.vs.alica.planDesigner.events.ModelEventType;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelManager;
import de.unikassel.vs.alica.planDesigner.modelmanagement.ModelModificationQuery;
import de.unikassel.vs.alica.planDesigner.modelmanagement.Types;

public class CreateTask extends Command {
    Task task;
    TaskRepository taskRepository;

    public CreateTask(ModelManager modelManager, ModelModificationQuery mmq) {
        super(modelManager, mmq);
        this.taskRepository = (TaskRepository) modelManager.getPlanElement(mmq.getParentId());
        this.task = createTask(this.taskRepository);
    }

    protected Task createTask(TaskRepository taskRepository) {
        Task task = new Task();
        task.setName(mmq.getName());
        task.setTaskRepository(taskRepository);
        return task;
    }

    @Override
    public void doCommand() {
        this.taskRepository.addTask(task);
        modelManager.storePlanElement(Types.TASK, this.task,  false);
        this.fireEvent(ModelEventType.ELEMENT_CREATED_AND_ADDED, this.task);

        // Save the new Task directly
        PlanElement planElement = modelManager.getPlanElement(task.getTaskRepository().getId());
        this.modelManager.serialize((SerializablePlanElement) planElement, mmq.getElementType());
    }

    @Override
    public void undoCommand() {
        this.taskRepository.removeTask(task);
        modelManager.dropPlanElement(Types.TASK, this.task, false);
        this.fireEvent(ModelEventType.ELEMENT_REMOVED_AND_DELETED, this.task);
    }
}
