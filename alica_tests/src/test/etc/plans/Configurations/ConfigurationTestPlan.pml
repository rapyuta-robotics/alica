{
  "id" : 1588060981661,
  "name" : "ConfigurationTestPlan",
  "comment" : "",
  "relativeDirectory" : "Configurations",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1588061024407,
    "name" : "1588061024407",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1588060991102,
    "plan" : 1588060981661
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1588060991102,
    "name" : "Default Name",
    "comment" : "",
    "entryPoint" : 1588061024407,
    "parentPlan" : 1588060981661,
    "confAbstractPlanWrappers" : [ {
      "id" : 1588061141112,
      "name" : "1588061141112",
      "comment" : "",
      "abstractPlan" : "Behaviour/ReadConfigurationBehaviour.beh#1588061129360",
      "configuration" : "Configurations/ConfigA.cfg#1588061188681"
    }, {
      "id" : 1588061144175,
      "name" : "1588061144175",
      "comment" : "",
      "abstractPlan" : "Behaviour/ReadConfigurationBehaviour.beh#1588061129360",
      "configuration" : "Configurations/ConfigB.cfg#1588061200689"
    }, {
      "id" : 1588246105794,
      "name" : "1588246105794",
      "comment" : "",
      "abstractPlan" : "Configurations/ReadConfigurationPlantype.pty#1588061351007",
      "configuration" : "Configurations/ConfigA.cfg#1588061188681"
    }, {
      "id" : 1588253325052,
      "name" : "1588253325052",
      "comment" : "",
      "abstractPlan" : "Configurations/ReadConfigurationPlan.pml#1588061334567",
      "configuration" : "Configurations/ConfigA.cfg#1588061188681"
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1588253347211 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1588253341545,
    "name" : "Default Name",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1588060981661,
    "confAbstractPlanWrappers" : [ {
      "id" : 1588253360557,
      "name" : "1588253360557",
      "comment" : "",
      "abstractPlan" : "Configurations/ReadConfigurationPlantype.pty#1588061351007",
      "configuration" : "Configurations/ConfigB.cfg#1588061200689"
    }, {
      "id" : 1588253370222,
      "name" : "1588253370222",
      "comment" : "",
      "abstractPlan" : "Configurations/ReadConfigurationPlan.pml#1588061334567",
      "configuration" : "Configurations/ConfigB.cfg#1588061200689"
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1588253347211 ]
  } ],
  "transitions" : [ {
    "id" : 1588253347211,
    "name" : "FromDefault NameTo Default Name",
    "comment" : "MISSING_COMMENT",
    "inState" : 1588060991102,
    "outState" : 1588253341545,
    "preCondition" : {
      "id" : 1588253347213,
      "name" : "1588253347213",
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