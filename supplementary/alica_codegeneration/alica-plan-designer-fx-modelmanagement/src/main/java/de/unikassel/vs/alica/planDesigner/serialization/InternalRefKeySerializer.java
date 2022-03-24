package de.unikassel.vs.alica.planDesigner.serialization;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.JsonSerializer;
import com.fasterxml.jackson.databind.SerializerProvider;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;

import java.io.IOException;

public class InternalRefKeySerializer extends JsonSerializer<PlanElement> {

    @Override
    public void serialize(PlanElement value, JsonGenerator gen, SerializerProvider serializers) throws IOException {
        if (value != null) {
            gen.writeFieldName(String.valueOf(value.getId()));
        } else {
            gen.writeFieldName("");
        }
    }
}
