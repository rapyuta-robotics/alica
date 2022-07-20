package de.unikassel.vs.alica.planDesigner.deserialization;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.TreeNode;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.node.ValueNode;
import de.unikassel.vs.alica.planDesigner.alicamodel.ConditionRepository;

import java.io.IOException;

public class ConditionRepositoryDeserializer extends StdDeserializer<ConditionRepository> {

    public ConditionRepositoryDeserializer() {
        this(null);
    }

    public ConditionRepositoryDeserializer(Class<?> vc) {
        super(vc);
    }

    @Override
    public ConditionRepository deserialize(JsonParser jsonparser, DeserializationContext context) throws IOException {
        TreeNode tree = jsonparser.getCodec().readTree(jsonparser);
        String conditionRepoString = ((ValueNode) tree).asText();
        int idIndex = conditionRepoString.indexOf('#');
        conditionRepoString = conditionRepoString.substring(idIndex + 1);
        ConditionRepository conditionRepository = new ConditionRepository(Long.parseLong(conditionRepoString));
        return conditionRepository;
    }
}
