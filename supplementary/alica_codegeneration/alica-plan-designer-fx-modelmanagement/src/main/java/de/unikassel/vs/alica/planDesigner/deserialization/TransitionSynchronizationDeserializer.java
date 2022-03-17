package de.unikassel.vs.alica.planDesigner.deserialization;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.TreeNode;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.node.LongNode;
import de.unikassel.vs.alica.planDesigner.alicamodel.Synchronisation;

import java.io.IOException;


public class TransitionSynchronizationDeserializer extends StdDeserializer<Synchronisation> {

        public TransitionSynchronizationDeserializer() {
                this(null);
        }

        public TransitionSynchronizationDeserializer(Class<?> vc) {super(vc);}

        @Override
        public Synchronisation deserialize(
                JsonParser jsonparser,
                DeserializationContext context)
                throws IOException, JsonProcessingException {
                TreeNode tree = jsonparser.getCodec().readTree(jsonparser);
                return new Synchronisation(((LongNode)tree).longValue());
        }
}
