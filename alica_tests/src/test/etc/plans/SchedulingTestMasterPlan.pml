{
  "id" : 1613378382024,
  "name" : "SchedulingTestMasterPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1613378485232,
    "name" : "1613378485232",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613371619454",
    "state" : 1613378474109,
    "plan" : 1613378382024
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1613378474109,
    "name" : "Default Name",
    "comment" : "",
    "entryPoint" : 1613378485232,
    "parentPlan" : 1613378382024,
    "confAbstractPlanWrappers" : [ {
      "id" : 1613378494690,
      "name" : "1613378494690",
      "comment" : "",
      "abstractPlan" : "SchedulingTestPlan1.pml#1613378406860",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1613530643879 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1613530614559,
    "name" : "EndTest",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1613378382024,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1613530643879 ]
  } ],
  "transitions" : [ {
    "id" : 1613530643879,
    "name" : "FromDefault NameTo EndTest",
    "comment" : "MISSING_COMMENT",
    "inState" : 1613378474109,
    "outState" : 1613530614559,
    "preCondition" : {
      "id" : 1613530643882,
      "name" : "1613530643882",
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