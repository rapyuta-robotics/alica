{
  "id" : 1418042796751,
  "name" : "PreConditionPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : {
    "id" : 1418042929966,
    "name" : "NewPreCondition",
    "comment" : "",
    "enabled" : true,
    "conditionString" : "Test",
    "pluginName" : "DefaultPlugin",
    "variables" : [ ],
    "quantifiers" : [ ]
  },
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1418042796753,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1418042796752,
    "plan" : 1418042796751
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1418042796752,
    "name" : "PreConditionTest",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1418042796751,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}