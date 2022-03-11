package de.unikassel.vs.alica.planDesigner.deserialization;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.TreeNode;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.node.ValueNode;
import de.unikassel.vs.alica.planDesigner.alicamodel.AbstractPlan;

import java.io.IOException;

public class AbstractPlanDeserializer extends StdDeserializer<AbstractPlan> {

    public AbstractPlanDeserializer() {
        this(null);
    }

    public AbstractPlanDeserializer(Class<?> vc) {
        super(vc);
    }

    @Override
    public AbstractPlan deserialize(
            JsonParser jsonparser,
            DeserializationContext context)
            throws IOException, JsonProcessingException {
        TreeNode tree = jsonparser.getCodec().readTree(jsonparser);
        String planElementString = ((ValueNode) tree).asText().trim();
        int idIndex = planElementString.indexOf('#');
        planElementString = planElementString.substring(idIndex + 1);
        AbstractPlan planElement = new AbstractPlan(Long.parseLong(planElementString));
        return planElement;
    }
}
