{
  "id" : 1529456584982,
  "name" : "BackForth",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1529456584984,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1529456584983,
    "plan" : 1529456584982
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1529456584983,
    "name" : "First",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1529456584982,
    "abstractPlans" : [ "Behaviour/CountIndefinitely.beh#1529456643148" ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1529456609989 ],
    "inTransitions" : [ 1529456610905 ]
  }, {
    "type" : "State",
    "id" : 1529456591410,
    "name" : "Second",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1529456584982,
    "abstractPlans" : [ "Behaviour/CountIndefinitely.beh#1529456643148" ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1529456610905 ],
    "inTransitions" : [ 1529456609989 ]
  } ],
  "transitions" : [ {
    "id" : 1529456609989,
    "name" : "MISSING_NAME",
    "comment" : "Forth",
    "inState" : 1529456584983,
    "outState" : 1529456591410,
    "preCondition" : {
      "id" : 1529456610697,
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
    "id" : 1529456610905,
    "name" : "MISSING_NAME",
    "comment" : "Back",
    "inState" : 1529456591410,
    "outState" : 1529456584983,
    "preCondition" : {
      "id" : 1529456611916,
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