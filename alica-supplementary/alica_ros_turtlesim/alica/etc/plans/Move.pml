{
  "id" : 1542882005838,
  "name" : "Move",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : {
    "id" : 1543284793605,
    "name" : "CircleRuntimeCondition",
    "comment" : "",
    "enabled" : false,
    "conditionString" : "",
    "pluginName" : "DefaultPlugin",
    "variables" : [ ],
    "quantifiers" : [ {
      "id" : 1543284852310,
      "name" : "MISSING_NAME",
      "comment" : "",
      "quantifierType" : "ALL",
      "scope" : 1542882005838,
      "sorts" : [ "x", "y" ]
    } ]
  },
  "entryPoints" : [ {
    "id" : 1543227886876,
    "name" : "LeaderTask",
    "comment" : "",
    "successRequired" : true,
    "minCardinality" : 1,
    "maxCardinality" : 1,
    "task" : "TaskRepository.tsk#1543224732214",
    "state" : 1542882041936,
    "plan" : 1542882005838
  }, {
    "id" : 1543227889789,
    "name" : "FollowerTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "TaskRepository.tsk#1543224747408",
    "state" : 1542882494678,
    "plan" : 1542882005838
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1542882041936,
    "name" : "Move2Center",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1542882005838,
    "confAbstractPlanWrappers" : [ {
      "id" : 1601278930757,
      "name" : "1601278930757",
      "comment" : "",
      "abstractPlan" : "Behaviours/GoTo.beh#1544160969061",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1542882494678,
    "name" : "AlignCircle",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1542882005838,
    "confAbstractPlanWrappers" : [ {
      "id" : 1601278930760,
      "name" : "1601278930760",
      "comment" : "",
      "abstractPlan" : "Behaviours/GoTo.beh#1544160969061",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}