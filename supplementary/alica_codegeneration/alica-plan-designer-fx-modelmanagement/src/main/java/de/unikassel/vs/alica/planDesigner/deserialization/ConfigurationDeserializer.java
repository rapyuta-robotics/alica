package de.unikassel.vs.alica.planDesigner.deserialization;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.TreeNode;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.node.ValueNode;
import de.unikassel.vs.alica.planDesigner.alicamodel.Configuration;

import java.io.IOException;

public class ConfigurationDeserializer extends StdDeserializer<Configuration> {

    public ConfigurationDeserializer() {
        this(null);
    }

    public ConfigurationDeserializer(Class<?> vc) {
        super(vc);
    }

    @Override
    public Configuration deserialize(
            JsonParser jsonparser,
            DeserializationContext context)
            throws IOException, JsonProcessingException {
        TreeNode tree = jsonparser.getCodec().readTree(jsonparser);
        String planElementString = ((ValueNode) tree).asText().trim();
        int idIndex = planElementString.indexOf('#');
        planElementString = planElementString.substring(idIndex + 1);
        Configuration planElement = new Configuration(Long.parseLong(planElementString));
        return planElement;
    }
}