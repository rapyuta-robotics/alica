{
  "id" : 1614964379654,
  "name" : "SchedulingTestSequenceSubPlan1",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1614964853637,
    "name" : "1614964853637",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613372009777",
    "state" : 1614964864614,
    "plan" : 1614964379654
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1614964864614,
    "name" : "InitSequenceSubPlan1",
    "comment" : "",
    "entryPoint" : 1614964853637,
    "parentPlan" : 1614964379654,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}