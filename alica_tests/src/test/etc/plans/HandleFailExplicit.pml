{
  "id" : 1530004915640,
  "name" : "HandleFailExplicit",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1530004915642,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1530004915641,
    "plan" : 1530004915640
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1530004915641,
    "name" : "A",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1530004915640,
    "abstractPlans" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1530004992551 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1530004973591,
    "name" : "B",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1530004915640,
    "abstractPlans" : [ "FailsOnOne.pml#1530069246103" ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1530004993680 ],
    "inTransitions" : [ 1530004992551 ]
  }, {
    "type" : "State",
    "id" : 1530004975275,
    "name" : "C",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1530004915640,
    "abstractPlans" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1532424092280 ],
    "inTransitions" : [ 1530004993680 ]
  }, {
    "type" : "State",
    "id" : 1532424087894,
    "name" : "D",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1530004915640,
    "abstractPlans" : [ "Behaviour/AlwaysFail.beh#1532424188199" ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1532424112331 ],
    "inTransitions" : [ 1532424092280 ]
  }, {
    "type" : "State",
    "id" : 1532424097662,
    "name" : "E",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1530004915640,
    "abstractPlans" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1532424112331 ]
  } ],
  "transitions" : [ {
    "id" : 1530004992551,
    "name" : "MISSING_NAME",
    "comment" : "From A to B, isset(0)",
    "inState" : 1530004915641,
    "outState" : 1530004973591,
    "preCondition" : {
      "id" : 1530004993493,
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
    "id" : 1530004993680,
    "name" : "MISSING_NAME",
    "comment" : "AnyChildFail",
    "inState" : 1530004973591,
    "outState" : 1530004975275,
    "preCondition" : {
      "id" : 1530004994611,
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
    "id" : 1532424092280,
    "name" : "MISSING_NAME",
    "comment" : "C to D, isset(2)",
    "inState" : 1530004975275,
    "outState" : 1532424087894,
    "preCondition" : {
      "id" : 1532424093178,
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
    "id" : 1532424112331,
    "name" : "MISSING_NAME",
    "comment" : "AnyChildFail",
    "inState" : 1532424087894,
    "outState" : 1532424097662,
    "preCondition" : {
      "id" : 1532424113475,
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