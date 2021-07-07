{
  "id" : 1614964444419,
  "name" : "SchedulingTestSequenceSubPlan2",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1614964892485,
    "name" : "1614964892485",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613372009777",
    "state" : 1614964894440,
    "plan" : 1614964444419
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1614964894440,
    "name" : "InitSequenceSubPlan1",
    "comment" : "",
    "entryPoint" : 1614964892485,
    "parentPlan" : 1614964444419,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}