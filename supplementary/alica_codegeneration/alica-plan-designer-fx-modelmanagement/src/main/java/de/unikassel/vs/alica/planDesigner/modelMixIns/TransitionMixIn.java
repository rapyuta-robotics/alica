package de.unikassel.vs.alica.planDesigner.modelMixIns;

import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import de.unikassel.vs.alica.planDesigner.alicamodel.PreCondition;
import de.unikassel.vs.alica.planDesigner.alicamodel.State;
import de.unikassel.vs.alica.planDesigner.alicamodel.Synchronisation;
import de.unikassel.vs.alica.planDesigner.deserialization.TransitionStateDeserializer;
import de.unikassel.vs.alica.planDesigner.deserialization.TransitionSynchronizationDeserializer;
import de.unikassel.vs.alica.planDesigner.serialization.InternalRefSerializer;

public abstract class TransitionMixIn {
    @JsonSerialize(using = InternalRefSerializer.class)
    @JsonDeserialize(using = TransitionStateDeserializer.class)
    protected State inState;
    @JsonSerialize(using = InternalRefSerializer.class)
    @JsonDeserialize(using = TransitionStateDeserializer.class)
    protected State outState;
    protected PreCondition preCondition;
    @JsonSerialize(using = InternalRefSerializer.class)
    @JsonDeserialize(using = TransitionSynchronizationDeserializer.class)
    protected Synchronisation synchronisation;
}
