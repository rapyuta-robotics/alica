package de.unikassel.vs.alica.planDesigner.deserialization;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.TreeNode;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.node.ValueNode;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;

import java.io.IOException;

public class PlanDeserializer extends StdDeserializer<Plan> {

    public PlanDeserializer() {
        this(null);
    }

    public PlanDeserializer(Class<?> vc) {
        super(vc);
    }

    @Override
    public Plan deserialize(
            JsonParser jsonparser,
            DeserializationContext context)
            throws IOException, JsonProcessingException {
        TreeNode tree = jsonparser.getCodec().readTree(jsonparser);
        String planString = ((ValueNode) tree).asText();
        int idIndex = planString.indexOf('#');
        planString = planString.substring(idIndex + 1);
        Plan plan = new Plan(Long.parseLong(planString));
        return plan;
    }
}
