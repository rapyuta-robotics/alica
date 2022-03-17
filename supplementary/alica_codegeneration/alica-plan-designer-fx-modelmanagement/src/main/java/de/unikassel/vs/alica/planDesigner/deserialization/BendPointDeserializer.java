package de.unikassel.vs.alica.planDesigner.deserialization;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.TreeNode;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.node.ValueNode;
import de.unikassel.vs.alica.planDesigner.alicamodel.Transition;

import java.io.IOException;

public class BendPointDeserializer extends StdDeserializer<Transition> {

    public BendPointDeserializer() {
        this(null);
    }

    public BendPointDeserializer(Class<?> vc) {
        super(vc);
    }

    @Override
    public Transition deserialize(JsonParser jsonParser, DeserializationContext deserializationContext) throws IOException, JsonProcessingException {
        TreeNode tree = jsonParser.getCodec().readTree(jsonParser);
        String transitionString = ((ValueNode) tree).asText();
        int idIndex = transitionString.indexOf('#');
        transitionString = transitionString.substring(idIndex + 1);
        Transition transition = new Transition(Long.parseLong(transitionString));
        return transition;
    }
}
