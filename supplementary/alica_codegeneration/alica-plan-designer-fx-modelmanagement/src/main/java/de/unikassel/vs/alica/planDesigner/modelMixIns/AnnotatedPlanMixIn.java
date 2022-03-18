package de.unikassel.vs.alica.planDesigner.modelMixIns;

import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan;
import de.unikassel.vs.alica.planDesigner.deserialization.PlanDeserializer;
import de.unikassel.vs.alica.planDesigner.serialization.ExternalRefSerializer;

public abstract class AnnotatedPlanMixIn {
    @JsonSerialize(using = ExternalRefSerializer.class)
    @JsonDeserialize(using = PlanDeserializer.class)
    private Plan plan;
}
