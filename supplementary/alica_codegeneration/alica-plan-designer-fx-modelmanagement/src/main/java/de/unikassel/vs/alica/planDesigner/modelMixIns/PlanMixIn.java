package de.unikassel.vs.alica.planDesigner.modelMixIns;

import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import de.unikassel.vs.alica.planDesigner.deserialization.SimpleLongPropertyDeserializer;
import javafx.beans.property.SimpleLongProperty;

public abstract class PlanMixIn {
    @JsonDeserialize(using = SimpleLongPropertyDeserializer.class)
    protected SimpleLongProperty id;
}
