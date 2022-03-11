package de.unikassel.vs.alica.planDesigner.alicamodel;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

public class Role extends PlanElement {

    protected HashMap<Task, Float> taskPriorities = new HashMap<>();
    protected ArrayList<Characteristic> characteristics = new ArrayList<>();
    private RoleSet roleSet;
    private ChangeListenerForDirtyFlag attributeChangedListener;

    public Role() {
        super();
    }
    public Role(long id) {
        this.id = id;
    }

    public Float getPriority(long taskID) {
        Optional<Task> task = taskPriorities.keySet().stream().filter(t -> t.getId() == taskID).findFirst();
        return task.isPresent()? taskPriorities.get(task.get()): null;
    }
    public Float getPriority(Task task) {return taskPriorities.get(task); }

    public void setTaskPriorities(HashMap<Task, Float> taskPriorities) {
        this.taskPriorities = taskPriorities;
//        this.attributeChangedListener.setDirty();
    }

    public HashMap<Task, Float> getTaskPriorities() {
        return this.taskPriorities;
    }

    public void removeTaskPriority(Task task) {
        this.taskPriorities.remove(task);
        if (attributeChangedListener != null) {
            this.attributeChangedListener.setDirty();
        }
    }

    public void addTaskPriority(Task task, float priority) {
        this.taskPriorities.put(task, priority);
        if (attributeChangedListener != null) {
            this.attributeChangedListener.setDirty();
        }
    }

    public ArrayList<Characteristic> getCharacteristics() {
        return this.characteristics;
    }
    public void setCharacteristics(ArrayList<Characteristic> characteristics) {
        this.characteristics = characteristics;
    }

    public RoleSet getRoleSet() {
        return roleSet;
    }
    public void setRoleSet(RoleSet roleSet) {
        this.roleSet = roleSet;
    }

    public void registerDirtyFlag(ChangeListenerForDirtyFlag listener) {
        this.attributeChangedListener = listener;
        this.name.addListener(listener);
        this.comment.addListener(listener);
        this.characteristics.forEach(c -> c.addListener(listener));
    }

    public Characteristic getCharacteristic(String characteristicName) {

        for (Characteristic characteristic : characteristics) {

            if (characteristic.getName().equals(characteristicName))
                return characteristic;
        }
        return null;
    }
    public void addCharacteristic(Characteristic characteristic) {
        this.characteristics.add(characteristic);
    }

    public void removeCharacteristic(Characteristic characteristic) {
        characteristic.setRole(null);
        this.characteristics.remove(characteristic);
    }
}
