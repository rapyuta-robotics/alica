package de.unikassel.vs.alica.planDesigner.alicamodel;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class TaskRepository extends SerializablePlanElement {

    protected final ArrayList<Task> tasks = new ArrayList<>();

    public TaskRepository() {}

    public TaskRepository(long id) { this.id = id; }

    public void addTask(Task task) {
        if (task.getTaskRepository() != this) {
            task.setTaskRepository(this);
        }
        task.registerDirtyFlag(this.changeListenerForDirtyFlag);
        tasks.add(task);
        this.changeListenerForDirtyFlag.setDirty();
    }
    public void removeTask(Task task) {
        tasks.remove(task);
        task.setTaskRepository(null);
        this.changeListenerForDirtyFlag.setDirty();
    }
    public List<Task> getTasks() {
        return Collections.unmodifiableList(tasks);
    }

    public Task getTask(long taskID) {
        for (Task task : tasks) {
            if (taskID == task.getId()) {
                return task;
            }
        }
        return null;
    }

    public boolean contains (Task task) {
        return tasks.contains(task);
    }

    @Override
    public void registerDirtyFlag() {
        super.registerDirtyFlag();
        for (Task task : tasks) {
            task.registerDirtyFlag(this.changeListenerForDirtyFlag);
        }
    }
}
