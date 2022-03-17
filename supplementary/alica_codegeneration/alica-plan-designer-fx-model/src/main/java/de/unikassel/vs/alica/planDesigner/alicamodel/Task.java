package de.unikassel.vs.alica.planDesigner.alicamodel;

public class Task extends PlanElement {

    protected TaskRepository taskRepository;

    public Task() {
        super();
    }

    public Task (long id) {
        this.id = id;
    }

    public TaskRepository getTaskRepository() {
        return taskRepository;
    }

    public void setTaskRepository(TaskRepository taskRepository) {
        this.taskRepository = taskRepository;
    }

    public void registerDirtyFlag(ChangeListenerForDirtyFlag listener) {
        comment.addListener(listener);
        name.addListener(listener);
    }
}
