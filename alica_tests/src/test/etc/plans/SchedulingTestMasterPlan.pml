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
    "id" : 1615797283419,
    "name" : "1615797283419",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613371619454",
    "state" : 1615797271229,
    "plan" : 1613378382024
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1613378474109,
    "name" : "InitTest",
    "comment" : "",
    "entryPoint" : null,
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
    "inTransitions" : [ 1615797316170 ]
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
    "inTransitions" : [ 1613530643879, 1615797365363, 1629895607017 ]
  }, {
    "type" : "State",
    "id" : 1615797271229,
    "name" : "Default Name",
    "comment" : "",
    "entryPoint" : 1615797283419,
    "parentPlan" : 1613378382024,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1615797316170, 1615797327076, 1629895598464 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1615797319003,
    "name" : "Default Name",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1613378382024,
    "confAbstractPlanWrappers" : [ {
      "id" : 1615797379906,
      "name" : "1615797379906",
      "comment" : "",
      "abstractPlan" : "SchedulingTestSequencePlan1.pml#1614963946725",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1615797365363 ],
    "inTransitions" : [ 1615797327076 ]
  }, {
    "type" : "State",
    "id" : 1629895593451,
    "name" : "OrderedSchedulingTestPlan",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1613378382024,
    "confAbstractPlanWrappers" : [ {
      "id" : 1629895663156,
      "name" : "1629895663156",
      "comment" : "",
      "abstractPlan" : "OrderedSchedulingTestPlan.pml#1629895582410",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1629895607017 ],
    "inTransitions" : [ 1629895598464 ]
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
  }, {
    "id" : 1615797316170,
    "name" : "FromDefault NameTo InitTest",
    "comment" : "MISSING_COMMENT",
    "inState" : 1615797271229,
    "outState" : 1613378474109,
    "preCondition" : {
      "id" : 1615797316171,
      "name" : "1615797316171",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1615797327076,
    "name" : "FromDefault NameTo Default Name",
    "comment" : "MISSING_COMMENT",
    "inState" : 1615797271229,
    "outState" : 1615797319003,
    "preCondition" : {
      "id" : 1615797327077,
      "name" : "1615797327077",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1615797365363,
    "name" : "FromDefault NameTo EndTest",
    "comment" : "MISSING_COMMENT",
    "inState" : 1615797319003,
    "outState" : 1613530614559,
    "preCondition" : {
      "id" : 1615797365364,
      "name" : "1615797365364",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1629895598464,
    "name" : "FromDefault NameTo Default Name",
    "comment" : "MISSING_COMMENT",
    "inState" : 1615797271229,
    "outState" : 1629895593451,
    "preCondition" : {
      "id" : 1629895598471,
      "name" : "1629895598471",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1629895607017,
    "name" : "FromDefault NameTo EndTest",
    "comment" : "MISSING_COMMENT",
    "inState" : 1629895593451,
    "outState" : 1613530614559,
    "preCondition" : {
      "id" : 1629895607018,
      "name" : "1629895607018",
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