{
  "id" : 1402488870347,
  "name" : "GoalPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ {
    "id" : 1403773747758,
    "name" : "test",
    "comment" : "",
    "variableType" : "test"
  } ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : {
    "id" : 1402489131988,
    "name" : "PreCondition",
    "comment" : "",
    "enabled" : true,
    "conditionString" : "",
    "pluginName" : "DefaultPlugin",
    "variables" : [ ],
    "quantifiers" : [ ]
  },
  "runtimeCondition" : {
    "id" : 1403773741874,
    "name" : "NewRuntimeCondition",
    "comment" : "",
    "enabled" : false,
    "conditionString" : "test",
    "pluginName" : "DefaultPlugin",
    "variables" : [ 1403773747758 ],
    "quantifiers" : [ {
      "id" : 1403773772633,
      "name" : "MISSING_NAME",
      "comment" : "",
      "quantifierType" : "ALL",
      "scope" : 1402489152217,
      "sorts" : [ "test" ]
    } ]
  },
  "entryPoints" : [ {
    "id" : 1402488881800,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1402488881799,
    "plan" : 1402488870347
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1402488881799,
    "name" : "Shoot",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488870347,
    "abstractPlans" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1402489173167 ],
    "inTransitions" : [ 1402489205153 ]
  }, {
    "type" : "State",
    "id" : 1402489152217,
    "name" : "Miss",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488870347,
    "abstractPlans" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1402489205153, 1402489216617 ],
    "inTransitions" : [ 1402489173167 ]
  }, {
    "type" : "TerminalState",
    "id" : 1402489192198,
    "name" : "Scored",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488870347,
    "abstractPlans" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1402489216617 ],
    "success" : true,
    "postCondition" : {
      "id" : 1402489620773,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : false,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    }
  } ],
  "transitions" : [ {
    "id" : 1402489173167,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1402488881799,
    "outState" : 1402489152217,
    "preCondition" : {
      "id" : 1402489174338,
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
    "id" : 1402489205153,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1402489152217,
    "outState" : 1402488881799,
    "preCondition" : {
      "id" : 1402489206278,
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
    "id" : 1402489216617,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1402489152217,
    "outState" : 1402489192198,
    "preCondition" : {
      "id" : 1402489218027,
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