{
  "id" : 1629895582410,
  "name" : "OrderedSchedulingTestPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1629895738193,
    "name" : "1629895738193",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613371619454",
    "state" : 1629895681520,
    "plan" : 1629895582410
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1629895681520,
    "name" : "PlanA",
    "comment" : "",
    "entryPoint" : 1629895738193,
    "parentPlan" : 1629895582410,
    "confAbstractPlanWrappers" : [ {
      "id" : 1629895930356,
      "name" : "1629895930356",
      "comment" : "",
      "abstractPlan" : "PlanA.pml#1629895837159",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1629895758611 ],
    "inTransitions" : [ 1629895768181 ]
  }, {
    "type" : "State",
    "id" : 1629895684249,
    "name" : "PlanB",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1629895582410,
    "confAbstractPlanWrappers" : [ {
      "id" : 1629895934743,
      "name" : "1629895934743",
      "comment" : "",
      "abstractPlan" : "PlanB.pml#1629895853508",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1629895768181 ],
    "inTransitions" : [ 1629895758611 ]
  } ],
  "transitions" : [ {
    "id" : 1629895758611,
    "name" : "FromPlanATo PlanB",
    "comment" : "MISSING_COMMENT",
    "inState" : 1629895681520,
    "outState" : 1629895684249,
    "preCondition" : {
      "id" : 1629895758612,
      "name" : "1629895758612",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1629895768181,
    "name" : "FromPlanBTo PlanA",
    "comment" : "MISSING_COMMENT",
    "inState" : 1629895684249,
    "outState" : 1629895681520,
    "preCondition" : {
      "id" : 1629895768182,
      "name" : "1629895768182",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  } ],
  "synchronisations" : [ ]
}