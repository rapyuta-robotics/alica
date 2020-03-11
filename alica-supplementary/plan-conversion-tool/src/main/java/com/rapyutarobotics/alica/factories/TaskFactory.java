package com.rapyutarobotics.alica.factories;

import com.rapyutarobotics.alica.Tags;
import de.unikassel.vs.alica.planDesigner.alicamodel.Task;
import org.w3c.dom.Element;

public class TaskFactory extends Factory {

    public static Task create (Element taskNode) {
        Task task = new Task();
        Factory.setAttributes(taskNode, task);
        // the description field is obsolete -> use comments field
        if (task.getComment().isEmpty()) {
            task.setComment(taskNode.getAttribute(Tags.DESCRIPTION));
        }
        conversionTool.planElements.put(task.getId(), task);
        return task;
    }
}
