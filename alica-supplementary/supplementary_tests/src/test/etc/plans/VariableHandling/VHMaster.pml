{
  "id" : 1524452721452,
  "name" : "VHMaster",
  "comment" : "",
  "relativeDirectory" : "VariableHandling",
  "variables" : [ {
    "id" : 1524463022262,
    "name" : "MA",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1524463028066,
    "name" : "MB",
    "comment" : "",
    "variableType" : ""
  } ],
  "masterPlan" : true,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : {
    "id" : 1524463006078,
    "name" : "NewRuntimeCondition",
    "comment" : "Unrelated Condition",
    "enabled" : false,
    "conditionString" : "VHMaster Runtime Condition",
    "pluginName" : "DefaultPlugin",
    "variables" : [ 1524463022262, 1524463028066 ],
    "quantifiers" : [ ]
  },
  "entryPoints" : [ {
    "id" : 1524452721454,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1524452721453,
    "plan" : 1524452721452
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1524452721453,
    "name" : "NewState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1524452721452,
    "abstractPlans" : [ "VariableHandling/Lvl1.pml#1524452759599" ],
    "variableBindings" : [ {
      "id" : 1524463056023,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1524463022262,
      "subPlan" : "VariableHandling/Lvl1.pml#1524452759599",
      "subVariable" : "VariableHandling/Lvl1.pml#1524453336548"
    } ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}