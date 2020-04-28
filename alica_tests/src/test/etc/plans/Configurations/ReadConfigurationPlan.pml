{
  "id" : 1588061334567,
  "name" : "ReadConfigurationPlan",
  "comment" : "",
  "relativeDirectory" : "Configurations",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1588069183324,
    "name" : "1588069183324",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1588069177860,
    "plan" : 1588061334567
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1588069177860,
    "name" : "DecisionState",
    "comment" : "",
    "entryPoint" : 1588069183324,
    "parentPlan" : 1588061334567,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1588069612659, 1588069615552 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1588069261047,
    "name" : "StateA",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1588061334567,
    "confAbstractPlanWrappers" : [ {
      "id" : 1588069962503,
      "name" : "1588069962503",
      "comment" : "",
      "abstractPlan" : "Behaviour/ReadConfigurationBehaviour.beh#1588061129360",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1588069612659 ]
  }, {
    "type" : "State",
    "id" : 1588069265377,
    "name" : "StateB",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1588061334567,
    "confAbstractPlanWrappers" : [ {
      "id" : 1588069965338,
      "name" : "1588069965338",
      "comment" : "",
      "abstractPlan" : "Behaviour/ReadConfigurationBehaviour.beh#1588061129360",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1588069615552 ]
  } ],
  "transitions" : [ {
    "id" : 1588069612659,
    "name" : "FromDefault NameTo Default Name",
    "comment" : "MISSING_COMMENT",
    "inState" : 1588069177860,
    "outState" : 1588069261047,
    "preCondition" : {
      "id" : 1588069612661,
      "name" : "1588069612661",
      "comment" : "TestValue = 1",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1588069615552,
    "name" : "FromDefault NameTo Default Name",
    "comment" : "MISSING_COMMENT",
    "inState" : 1588069177860,
    "outState" : 1588069265377,
    "preCondition" : {
      "id" : 1588069615553,
      "name" : "1588069615553",
      "comment" : "TestValue = 2",
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