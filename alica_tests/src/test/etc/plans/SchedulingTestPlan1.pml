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
    "name" : "InitPlan1",
    "comment" : "",
    "entryPoint" : 1613378541158,
    "parentPlan" : 1613378406860,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1614960055819 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1613977406218,
    "name" : "TerminateSubPlans",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1613378406860,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1614960063842 ]
  }, {
    "type" : "State",
    "id" : 1614960038398,
    "name" : "InitSubPlans",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1613378406860,
    "confAbstractPlanWrappers" : [ {
      "id" : 1614960077329,
      "name" : "1614960077329",
      "comment" : "",
      "abstractPlan" : "SchedulingTestPlan2.pml#1613378423610",
      "configuration" : null
    }, {
      "id" : 1614960079819,
      "name" : "1614960079819",
      "comment" : "",
      "abstractPlan" : "SchedulingTestPlan3.pml#1613378433623",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1614960063842 ],
    "inTransitions" : [ 1614960055819 ]
  } ],
  "transitions" : [ {
    "id" : 1614960055819,
    "name" : "FromDefault NameTo Default Name",
    "comment" : "MISSING_COMMENT",
    "inState" : 1613378543512,
    "outState" : 1614960038398,
    "preCondition" : {
      "id" : 1614960055821,
      "name" : "1614960055821",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1614960063842,
    "name" : "FromDefault NameTo Default Name",
    "comment" : "MISSING_COMMENT",
    "inState" : 1614960038398,
    "outState" : 1613977406218,
    "preCondition" : {
      "id" : 1614960063843,
      "name" : "1614960063843",
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