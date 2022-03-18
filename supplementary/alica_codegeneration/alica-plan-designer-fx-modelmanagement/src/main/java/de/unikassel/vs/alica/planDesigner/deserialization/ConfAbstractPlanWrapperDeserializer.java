package de.unikassel.vs.alica.planDesigner.deserialization;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.TreeNode;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.databind.node.ValueNode;
import de.unikassel.vs.alica.planDesigner.alicamodel.AbstractPlan;
import de.unikassel.vs.alica.planDesigner.alicamodel.ConfAbstractPlanWrapper;

import java.io.IOException;

public class ConfAbstractPlanWrapperDeserializer extends StdDeserializer<ConfAbstractPlanWrapper> {

    public ConfAbstractPlanWrapperDeserializer() {
        this(null);
    }

    public ConfAbstractPlanWrapperDeserializer(Class<?> vc) {
        super(vc);
    }

    @Override
    public ConfAbstractPlanWrapper deserialize(
            JsonParser jsonparser,
            DeserializationContext context)
            throws IOException, JsonProcessingException {
        TreeNode tree = jsonparser.getCodec().readTree(jsonparser);
        if (tree instanceof ObjectNode) {
            // [new] type: objectNode
            JsonParser tmpParser = tree.traverse(jsonparser.getCodec());
            return tmpParser.getCodec().readValue(tmpParser, ConfAbstractPlanWrapper.class);
        } else {
            // [old] type: textNode, content: "abstractPlans" : [ "TestPlanType.pty#1418042702402" ],
            // TODO read old format...
            String planElementString = ((ValueNode) tree).asText().trim();
            int idIndex = planElementString.indexOf('#');
            planElementString = planElementString.substring(idIndex + 1);
            AbstractPlan abstractPlan = new AbstractPlan(Long.parseLong(planElementString));
            ConfAbstractPlanWrapper confAbstractPlanWrapper = new ConfAbstractPlanWrapper();
            confAbstractPlanWrapper.setAbstractPlan(abstractPlan);
            return confAbstractPlanWrapper;
        }
    }
}
