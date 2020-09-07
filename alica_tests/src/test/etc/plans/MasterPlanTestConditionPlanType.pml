{
  "id" : 1418042656594,
  "name" : "MasterPlanTestConditionPlanType",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1418042656596,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1418042656595,
    "plan" : 1418042656594
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1418042656595,
    "name" : "Start",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1418042656594,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1418042682960 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1418042674811,
    "name" : "Plantype",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1418042656594,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662539,
      "name" : "1587718662539",
      "comment" : "",
      "abstractPlan" : "TestPlanType.pty#1418042702402",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1418042682960 ]
  } ],
  "transitions" : [ {
    "id" : 1418042682960,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1418042656595,
    "outState" : 1418042674811,
    "preCondition" : {
      "id" : 1418042683692,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  } ],
  "synchronisations" : [ ]
}