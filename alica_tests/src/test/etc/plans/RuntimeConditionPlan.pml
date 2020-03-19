{
  "id" : 1418042806575,
  "name" : "RuntimeConditionPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : {
    "id" : 1418042967134,
    "name" : "NewRuntimeCondition",
    "comment" : "",
    "enabled" : false,
    "conditionString" : "",
    "pluginName" : "DefaultPlugin",
    "variables" : [ ],
    "quantifiers" : [ ]
  },
  "entryPoints" : [ {
    "id" : 1418042806577,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1418042806576,
    "plan" : 1418042806575
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1418042806576,
    "name" : "RuntimeConditionTest",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1418042806575,
    "abstractPlans" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}