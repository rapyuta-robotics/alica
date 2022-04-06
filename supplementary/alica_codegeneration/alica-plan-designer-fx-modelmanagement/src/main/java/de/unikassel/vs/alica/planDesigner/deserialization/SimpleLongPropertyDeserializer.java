package de.unikassel.vs.alica.planDesigner.deserialization;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.TreeNode;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.node.LongNode;
import com.fasterxml.jackson.databind.node.ValueNode;
import javafx.beans.property.SimpleLongProperty;

import java.io.IOException;

public class SimpleLongPropertyDeserializer extends StdDeserializer<SimpleLongProperty> {

    public SimpleLongPropertyDeserializer() {
        this(null);
    }

    public SimpleLongPropertyDeserializer(Class<?> vc) {
        super(vc);
    }

    @Override
    public SimpleLongProperty deserialize(
            JsonParser jsonparser,
            DeserializationContext context)
            throws IOException, JsonProcessingException {
        TreeNode tree = jsonparser.getCodec().readTree(jsonparser);
        return new SimpleLongProperty(((LongNode)tree).longValue());
    }
}
