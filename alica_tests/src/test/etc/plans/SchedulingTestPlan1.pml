{
  "id" : 1613378406860,
  "name" : "SchedulingTestPlan1",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1613378541158,
    "name" : "1613378541158",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613372009777",
    "state" : 1613378543512,
    "plan" : 1613378406860
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1613378543512,
    "name" : "Default Name",
    "comment" : "",
    "entryPoint" : 1613378541158,
    "parentPlan" : 1613378406860,
    "confAbstractPlanWrappers" : [ {
      "id" : 1613378551978,
      "name" : "1613378551978",
      "comment" : "",
      "abstractPlan" : "SchedulingTestPlan3.pml#1613378433623",
      "configuration" : null
    }, {
      "id" : 1613528284808,
      "name" : "1613528284808",
      "comment" : "",
      "abstractPlan" : "SchedulingTestPlan2.pml#1613378423610",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1613977426633 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1613977406218,
    "name" : "Default Name",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1613378406860,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1613977426633 ]
  } ],
  "transitions" : [ {
    "id" : 1613977426633,
    "name" : "FromDefault NameTo Default Name",
    "comment" : "MISSING_COMMENT",
    "inState" : 1613378543512,
    "outState" : 1613977406218,
    "preCondition" : {
      "id" : 1613977426634,
      "name" : "1613977426634",
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