{
  "id" : 1524452836022,
  "name" : "Lvl3",
  "comment" : "",
  "relativeDirectory" : "VariableHandling",
  "variables" : [ {
    "id" : 1524453054226,
    "name" : "L3A",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1524453060294,
    "name" : "L3B",
    "comment" : "",
    "variableType" : ""
  } ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : {
    "id" : 1524452937477,
    "name" : "NewRuntimeCondition",
    "comment" : "",
    "enabled" : false,
    "conditionString" : "Lvl3 Runtime Condition",
    "pluginName" : "DefaultPlugin",
    "variables" : [ 1524453054226, 1524453060294 ],
    "quantifiers" : [ {
      "id" : 1524453019900,
      "name" : "MISSING_NAME",
      "comment" : "",
      "quantifierType" : "ALL",
      "scope" : 1524452836022,
      "sorts" : [ "X", "Y" ]
    } ]
  },
  "entryPoints" : [ {
    "id" : 1524452836024,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1524452836023,
    "plan" : 1524452836022
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1524452836023,
    "name" : "NewState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1524452836022,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}