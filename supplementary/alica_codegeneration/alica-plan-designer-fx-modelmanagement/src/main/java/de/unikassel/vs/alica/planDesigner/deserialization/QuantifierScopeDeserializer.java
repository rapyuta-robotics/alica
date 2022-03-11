package de.unikassel.vs.alica.planDesigner.deserialization;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.TreeNode;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.node.ValueNode;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;

import java.io.IOException;

public class QuantifierScopeDeserializer extends StdDeserializer<PlanElement> {


    protected QuantifierScopeDeserializer() {
        super((Class<?>) null);
    }

    @Override
    public PlanElement deserialize(JsonParser p, DeserializationContext ctxt) throws IOException {
        TreeNode tree = p.getCodec().readTree(p);
        return new PlanElement(((ValueNode)tree).asLong());
    }
}
