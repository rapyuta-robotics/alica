{
  "id" : 1524452793378,
  "name" : "Lvl2",
  "comment" : "",
  "relativeDirectory" : "VariableHandling",
  "variables" : [ {
    "id" : 1524453150187,
    "name" : "L2A",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1524453155043,
    "name" : "L2B",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1524453162883,
    "name" : "L2C",
    "comment" : "",
    "variableType" : ""
  } ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : {
    "id" : 1524453266123,
    "name" : "NewRuntimeCondition",
    "comment" : "",
    "enabled" : false,
    "conditionString" : "Lvl2 Runtime Condition",
    "pluginName" : "DefaultPlugin",
    "variables" : [ 1524453150187, 1524453155043, 1524453162883 ],
    "quantifiers" : [ {
      "id" : 1524453283559,
      "name" : "MISSING_NAME",
      "comment" : "",
      "quantifierType" : "all",
      "scope" : 1524453248579,
      "sorts" : [ "X", "Y" ]
    } ]
  },
  "entryPoints" : [ {
    "id" : 1524452793380,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1524452793379,
    "plan" : 1524452793378
  }, {
    "id" : 1524453238753,
    "name" : "AttackTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 1,
    "maxCardinality" : 1,
    "task" : "taskrepository.tsk#1407153522080",
    "state" : 1524453248579,
    "plan" : 1524452793378
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1524452793379,
    "name" : "NewState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1524452793378,
    "confAbstractPlanWrappers" : [ {
      "id" : 1597658636528,
      "name" : "1597658636528",
      "comment" : "",
      "abstractPlan" : "VariableHandling/Lvl3.pml#1524452836022",
      "configuration" : null
    } ],
    "variableBindings" : [ {
      "id" : 1524453170258,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1524453155043,
      "subPlan" : "VariableHandling/Lvl3.pml#1524452836022",
      "subVariable" : "VariableHandling/Lvl3.pml#1524453060294"
    }, {
      "id" : 1524453176349,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1524453150187,
      "subPlan" : "VariableHandling/Lvl3.pml#1524452836022",
      "subVariable" : "VariableHandling/Lvl3.pml#1524453054226"
    } ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1524453248579,
    "name" : "Dummy",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1524452793378,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}
