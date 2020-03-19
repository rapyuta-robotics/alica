{
  "id" : 1414068495566,
  "name" : "ConstraintTestMaster",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1414068495568,
    "name" : "1414068495568",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1414068495567,
    "plan" : 1414068495566
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1414068495567,
    "name" : "Start",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1414068495566,
    "abstractPlans" : [ "ConstraintTestPlan.pml#1414068524245" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}