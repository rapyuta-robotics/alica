{
  "id" : 1524452759599,
  "name" : "Lvl1",
  "comment" : "",
  "relativeDirectory" : "VariableHandling",
  "variables" : [ {
    "id" : 1524453326397,
    "name" : "L1A",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1524453331530,
    "name" : "L1B",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1524453336548,
    "name" : "L1C",
    "comment" : "",
    "variableType" : ""
  } ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : {
    "id" : 1524453470580,
    "name" : "NewRuntimeCondition",
    "comment" : "",
    "enabled" : false,
    "conditionString" : "Lvl1 Runtime Condition",
    "pluginName" : "DefaultPlugin",
    "variables" : [ 1524453326397, 1524453331530 ],
    "quantifiers" : [ ]
  },
  "entryPoints" : [ {
    "id" : 1524452759601,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1524453481856,
    "plan" : 1524452759599
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1524452759600,
    "name" : "NewState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1524452759599,
    "confAbstractPlanWrappers" : [ {
      "id" : 1597658636548,
      "name" : "1597658636548",
      "comment" : "",
      "abstractPlan" : "VariableHandling/VHPLanType.pty#1524452770528",
      "configuration" : null
    } ],
    "variableBindings" : [ {
      "id" : 1524453528169,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1524453331530,
      "subPlan" : "VariableHandling/VHPLanType.pty#1524452770528",
      "subVariable" : "VariableHandling/VHPLanType.pty#1524453441020"
    }, {
      "id" : 1524453534656,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1524453326397,
      "subPlan" : "VariableHandling/VHPLanType.pty#1524452770528",
      "subVariable" : "VariableHandling/VHPLanType.pty#1524453433443"
    } ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1524453490345 ]
  }, {
    "type" : "State",
    "id" : 1524453481856,
    "name" : "BeforeTrans",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1524452759599,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1524453490345 ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ {
    "id" : 1524453490345,
    "name" : "MISSING_NAME",
    "comment" : "Lvl1 Transition",
    "inState" : 1524453481856,
    "outState" : 1524452759600,
    "preCondition" : {
      "id" : 1524453491764,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ {
        "id" : 1524453546255,
        "name" : "MISSING_NAME",
        "comment" : "",
        "quantifierType" : "all",
        "scope" : 1524452759601,
        "sorts" : [ "X", "Y" ]
      } ]
    },
    "synchronisation" : null
  } ],
  "synchronisations" : [ ]
}
