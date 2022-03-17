package de.unikassel.vs.alica.planDesigner.deserialization;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.TreeNode;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.node.ValueNode;
import de.unikassel.vs.alica.planDesigner.alicamodel.TaskRepository;

import java.io.IOException;

public class TaskTaskRepositoryDeserializer extends StdDeserializer<TaskRepository> {

    public TaskTaskRepositoryDeserializer() {
        this(null);
    }

    public TaskTaskRepositoryDeserializer(Class<?> vc) {
        super(vc);
    }

    @Override
    public TaskRepository deserialize(JsonParser jsonparser, DeserializationContext context) throws IOException {
        TreeNode tree = jsonparser.getCodec().readTree(jsonparser);
        String taskRepoString = ((ValueNode) tree).asText();
        int idIndex = taskRepoString.indexOf('#');
        taskRepoString = taskRepoString.substring(idIndex + 1);
        TaskRepository taskRepository = new TaskRepository(Long.parseLong(taskRepoString));
        return taskRepository;
    }
}
