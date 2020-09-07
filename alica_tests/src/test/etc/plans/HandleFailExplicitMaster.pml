{
  "id" : 1530004940652,
  "name" : "HandleFailExplicitMaster",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1530004940654,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1530004940653,
    "plan" : 1530004940652
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1530004940653,
    "name" : "NewState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1530004940652,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662811,
      "name" : "1587718662811",
      "comment" : "",
      "abstractPlan" : "HandleFailExplicit.pml#1530004915640",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}