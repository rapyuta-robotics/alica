{
  "id" : 1614964478264,
  "name" : "SchedulingTestSequenceSubPlan3",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1614964917149,
    "name" : "1614964917149",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613372009777",
    "state" : 1614964919973,
    "plan" : 1614964478264
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1614964919973,
    "name" : "InitSequenceSubPlan3",
    "comment" : "",
    "entryPoint" : 1614964917149,
    "parentPlan" : 1614964478264,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}