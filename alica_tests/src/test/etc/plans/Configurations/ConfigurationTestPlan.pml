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
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}