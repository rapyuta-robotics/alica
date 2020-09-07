{
  "id" : 1588061801734,
  "name" : "ReadConfInPlantype",
  "comment" : "",
  "relativeDirectory" : "Configurations",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1588103719479,
    "name" : "1588103719479",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1588103714226,
    "plan" : 1588061801734
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1588103714226,
    "name" : "Default Name",
    "comment" : "",
    "entryPoint" : 1588103719479,
    "parentPlan" : 1588061801734,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1588246144840, 1588246141555 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1588246134801,
    "name" : "ConfA",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1588061801734,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1588246141555 ]
  }, {
    "type" : "State",
    "id" : 1588246136647,
    "name" : "ConfB",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1588061801734,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1588246144840 ]
  } ],
  "transitions" : [ {
    "id" : 1588246144840,
    "name" : "FromDefault NameTo Default Name",
    "comment" : "MISSING_COMMENT",
    "inState" : 1588103714226,
    "outState" : 1588246136647,
    "preCondition" : {
      "id" : 1588246144841,
      "name" : "1588246144841",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1588246141555,
    "name" : "FromDefault NameTo Default Name",
    "comment" : "MISSING_COMMENT",
    "inState" : 1588103714226,
    "outState" : 1588246134801,
    "preCondition" : {
      "id" : 1588246141557,
      "name" : "1588246141557",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  } ],
  "synchronisations" : [ ]
}