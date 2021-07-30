{
  "id" : 1626848999740,
  "name" : "FrequencyTestPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1626849024805,
    "name" : "1626849024805",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1626849027475,
    "plan" : 1626848999740
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1626849027475,
    "name" : "Default Name",
    "comment" : "",
    "entryPoint" : 1626849024805,
    "parentPlan" : 1626848999740,
    "confAbstractPlanWrappers" : [ {
      "id" : 1626849067526,
      "name" : "1626849067526",
      "comment" : "",
      "abstractPlan" : "Behaviour/EmptyBehaviour.beh#1625610857563",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}