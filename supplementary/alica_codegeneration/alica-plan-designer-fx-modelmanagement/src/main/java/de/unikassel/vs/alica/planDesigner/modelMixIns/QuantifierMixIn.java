package de.unikassel.vs.alica.planDesigner.modelMixIns;

import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import de.unikassel.vs.alica.planDesigner.alicamodel.PlanElement;
import de.unikassel.vs.alica.planDesigner.deserialization.QuantifierScopeDeserializer;
import de.unikassel.vs.alica.planDesigner.serialization.InternalRefSerializer;

public abstract class QuantifierMixIn {
    @JsonSerialize(using = InternalRefSerializer.class)
    @JsonDeserialize(using = QuantifierScopeDeserializer.class)
    protected PlanElement scope;
}
