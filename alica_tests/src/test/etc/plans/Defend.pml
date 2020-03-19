{
  "id" : 1402488893641,
  "name" : "Defend",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1402488903550,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1402488959965,
    "plan" : 1402488893641
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1402488903549,
    "name" : "Tackle",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488893641,
    "abstractPlans" : [ "Behaviour/Tackle.beh#1402488939130", "Tackle.pml#1402489318663" ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1402488991762 ],
    "inTransitions" : [ 1402488990761 ]
  }, {
    "type" : "TerminalState",
    "id" : 1402488910751,
    "name" : "GetGoal",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488893641,
    "abstractPlans" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1402489071510 ],
    "success" : false,
    "postCondition" : null
  }, {
    "type" : "State",
    "id" : 1402488959965,
    "name" : "GetBall",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488893641,
    "abstractPlans" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1402488990761, 1402489064693 ],
    "inTransitions" : [ 1402488991762 ]
  }, {
    "type" : "State",
    "id" : 1402489037735,
    "name" : "TryToDefendGoal",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488893641,
    "abstractPlans" : [ "PlanType.pty#1402489564599" ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1402489071510 ],
    "inTransitions" : [ 1402489064693 ]
  } ],
  "transitions" : [ {
    "id" : 1402488990761,
    "name" : "TackleToGetBall",
    "comment" : "",
    "inState" : 1402488959965,
    "outState" : 1402488903549,
    "preCondition" : {
      "id" : 1402488991641,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1402488991762,
    "name" : "TackleToGetBall",
    "comment" : "",
    "inState" : 1402488903549,
    "outState" : 1402488959965,
    "preCondition" : {
      "id" : 1402488993122,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1402489064693,
    "name" : "GetBallToTryToDefendGoal",
    "comment" : "",
    "inState" : 1402488959965,
    "outState" : 1402489037735,
    "preCondition" : {
      "id" : 1402489065962,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1402489071510,
    "name" : "TryToDefendGoalToGetGoal",
    "comment" : "",
    "inState" : 1402489037735,
    "outState" : 1402488910751,
    "preCondition" : {
      "id" : 1402489073613,
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