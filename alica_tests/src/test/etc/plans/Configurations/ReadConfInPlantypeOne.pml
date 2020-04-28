{
  "id" : 1588061801734,
  "name" : "ReadConfInPlantypeOne",
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
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}