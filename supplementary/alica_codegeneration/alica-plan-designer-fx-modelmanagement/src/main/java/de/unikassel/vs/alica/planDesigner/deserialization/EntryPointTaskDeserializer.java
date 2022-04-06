package de.unikassel.vs.alica.planDesigner.deserialization;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.TreeNode;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.node.ValueNode;
import de.unikassel.vs.alica.planDesigner.alicamodel.Task;

import java.io.IOException;

public class EntryPointTaskDeserializer extends StdDeserializer<Task> {

    public EntryPointTaskDeserializer() {
        this(null);
    }

    public EntryPointTaskDeserializer(Class<?> vc) {
        super(vc);
    }

    @Override
    public Task deserialize(JsonParser jsonparser, DeserializationContext context) throws IOException {
        TreeNode tree = jsonparser.getCodec().readTree(jsonparser);
        String taskString = ((ValueNode) tree).asText();
        int idIndex = taskString.indexOf('#');
        taskString = taskString.substring(idIndex + 1);
        Task task = new Task(Long.parseLong(taskString));
        return task;
    }
}
