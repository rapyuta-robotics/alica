package de.unikassel.vs.alica.planDesigner.modelMixIns;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import de.unikassel.vs.alica.planDesigner.alicamodel.*;

public class BehaviourMixIn  {
    @JsonIgnoreProperties({"quantifiers"})
    PostCondition postCondition;

    @JsonIgnoreProperties({"quantifiers"})
    RuntimeCondition runtimeCondition;

    @JsonIgnoreProperties({"quantifiers"})
    PreCondition preCondition;
}
