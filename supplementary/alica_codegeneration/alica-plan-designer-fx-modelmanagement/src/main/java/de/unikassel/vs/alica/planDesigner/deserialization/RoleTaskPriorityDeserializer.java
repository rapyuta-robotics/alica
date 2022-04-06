package de.unikassel.vs.alica.planDesigner.deserialization;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.JsonDeserializer;
import de.unikassel.vs.alica.planDesigner.alicamodel.Task;

import java.io.IOException;
import java.util.HashMap;

public class RoleTaskPriorityDeserializer extends JsonDeserializer {

    @Override
    public HashMap<Task, Float> deserialize (JsonParser jsonparser, DeserializationContext context) throws IOException {
        HashMap<Task, Float> taskProperties = new HashMap();
        HashMap taskPropertyObjects = jsonparser.getCodec().readValue(jsonparser, HashMap.class);

        taskPropertyObjects.forEach((taskObj, priorityObj) -> {
            String taskString = (String) taskObj;
            taskString = taskString.substring(taskString.indexOf('#') + 1);
            Task task = new Task(Long.parseLong(taskString));
            taskProperties.put(task, Float.valueOf(String.valueOf(priorityObj)));
        });
        return taskProperties;
    }
}
