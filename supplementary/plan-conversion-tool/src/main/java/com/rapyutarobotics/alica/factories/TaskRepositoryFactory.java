package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.ConversionProcess;
import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Task;
import de.unikassel.vs.alica.planDesigner.alicamodel.TaskRepository;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class TaskRepositoryFactory extends Factory {

    public static TaskRepository create(Element taskRepositoryNode, ConversionProcess cp) {
        TaskRepository taskRepository = new TaskRepository();
        Factory.setAttributes(taskRepositoryNode, taskRepository);
        cp.addElement(taskRepository);

        NodeList tasksList = taskRepositoryNode.getElementsByTagName(Tags.TASKS);
        for (int i = 0; i < tasksList.getLength(); i++) {
            Element taskNode = (Element) tasksList.item(i);
            Task task = TaskFactory.create(taskNode, cp);
            taskRepository.addTask(task);
            task.setTaskRepository(taskRepository);
        }

        return taskRepository;
    }
}