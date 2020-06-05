{
  "id" : 1414068524245,
  "name" : "ConstraintTestPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ {
    "id" : 1414068572540,
    "name" : "X",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1414068576620,
    "name" : "Y",
    "comment" : "",
    "variableType" : ""
  } ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : {
    "id" : 1414068566297,
    "name" : "NewRuntimeCondition",
    "comment" : "",
    "enabled" : false,
    "conditionString" : "",
    "pluginName" : "DefaultPlugin",
    "variables" : [ 1414068572540, 1414068576620 ],
    "quantifiers" : [ ]
  },
  "entryPoints" : [ {
    "id" : 1414068524247,
    "name" : "1414068524247",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1414068524246,
    "plan" : 1414068524245
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1414068524246,
    "name" : "constraintRunner",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1414068524245,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718663002,
      "name" : "1587718663002",
      "comment" : "",
      "abstractPlan" : "Behaviour/ConstraintUsingBehaviour.beh#1414068597716",
      "configuration" : null
    } ],
    "variableBindings" : [ {
      "id" : 1416488166139,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1414068576620,
      "subPlan" : "Behaviour/ConstraintUsingBehaviour.beh#1414068597716",
      "subVariable" : "Behaviour/ConstraintUsingBehaviour.beh#1416488161203"
    }, {
      "id" : 1416488172649,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1414068572540,
      "subPlan" : "Behaviour/ConstraintUsingBehaviour.beh#1414068597716",
      "subVariable" : "Behaviour/ConstraintUsingBehaviour.beh#1416487733086"
    } ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}