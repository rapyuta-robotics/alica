package de.unikassel.vs.alica.planDesigner.modelMixIns;

import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import de.unikassel.vs.alica.planDesigner.alicamodel.Transition;
import de.unikassel.vs.alica.planDesigner.serialization.InternalRefSerializer;

import java.util.ArrayList;

public abstract class SynchronizationMixIn {
    @JsonSerialize(using = InternalRefSerializer.class)
    protected ArrayList<Transition> synchedTransitions;
}
