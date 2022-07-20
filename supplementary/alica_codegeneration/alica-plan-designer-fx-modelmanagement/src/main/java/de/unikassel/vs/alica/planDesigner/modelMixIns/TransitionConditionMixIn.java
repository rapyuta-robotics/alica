package de.unikassel.vs.alica.planDesigner.modelMixIns;

import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import de.unikassel.vs.alica.planDesigner.alicamodel.ConditionRepository;
import de.unikassel.vs.alica.planDesigner.deserialization.ConditionRepositoryDeserializer;
import de.unikassel.vs.alica.planDesigner.serialization.InternalRefSerializer;

public abstract class TransitionConditionMixIn {
    @JsonSerialize(using = InternalRefSerializer.class)
    @JsonDeserialize(using = ConditionRepositoryDeserializer.class)
    protected ConditionRepository conditionRepository;
}
