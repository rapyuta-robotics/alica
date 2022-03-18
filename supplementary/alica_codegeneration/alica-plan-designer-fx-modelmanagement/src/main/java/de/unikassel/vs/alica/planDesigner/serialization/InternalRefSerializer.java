package de.unikassel.vs.alica.planDesigner.serialization;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.jsontype.TypeSerializer;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;

import java.io.IOException;

public class InternalRefSerializer extends StdSerializer<PlanElement> {

    public InternalRefSerializer() {
        this(null);
    }

    public InternalRefSerializer(Class<PlanElement> t) {
        super(t);
    }

    @Override
    public void serialize(PlanElement planElement, JsonGenerator jsonGenerator, SerializerProvider serializerProvider) throws IOException {
        jsonGenerator.writeNumber(planElement.getId());
    }

    @Override
    public void serializeWithType(PlanElement planElement, JsonGenerator jsonGenerator,  SerializerProvider provider, TypeSerializer typeSer)
            throws IOException {
        // This method is only needed because of the states. States have a type to differentiate between TerminalStates and
        // normal States. This type is simply ignored for serialization
        jsonGenerator.writeNumber(planElement.getId());
    }

}
