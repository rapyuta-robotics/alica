{
  "id" : 1588061815706,
  "name" : "ReadConfInPlantypeTwo",
  "comment" : "",
  "relativeDirectory" : "Configurations",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1588103730327,
    "name" : "1588103730327",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1588103732464,
    "plan" : 1588061815706
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1588103732464,
    "name" : "Default Name",
    "comment" : "",
    "entryPoint" : 1588103730327,
    "parentPlan" : 1588061815706,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}