{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1414068524247,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1414068524247",
      "plan": 1414068524245,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1414068524246,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1414068524245,
  "masterPlan": false,
  "name": "ConstraintTestPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": {
    "comment": "",
    "conditionString": "",
    "enabled": false,
    "id": 1414068566297,
    "name": "NewRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [],
    "variables": [
      1414068572540,
      1414068576620
    ]
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/ConstraintUsingBehaviour.beh#1414068597716",
          "comment": "",
          "configuration": null,
          "id": 1587718663002,
          "name": "1587718663002"
        }
      ],
      "entryPoint": 1414068524247,
      "id": 1414068524246,
      "inTransitions": [],
      "name": "constraintRunner",
      "outTransitions": [],
      "parentPlan": 1414068524245,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": [
        {
          "comment": "",
          "id": 1416488166139,
          "name": "MISSING_NAME",
          "subPlan": "behaviours/ConstraintUsingBehaviour.beh#1414068597716",
          "subVariable": "behaviours/ConstraintUsingBehaviour.beh#1416488161203",
          "variable": 1414068576620
        },
        {
          "comment": "",
          "id": 1416488172649,
          "name": "MISSING_NAME",
          "subPlan": "behaviours/ConstraintUsingBehaviour.beh#1414068597716",
          "subVariable": "behaviours/ConstraintUsingBehaviour.beh#1416487733086",
          "variable": 1414068572540
        }
      ]
    }
  ],
  "synchronisations": [],
  "transitions": [],
  "utilityThreshold": 0.1,
  "variables": [
    {
      "comment": "",
      "id": 1414068572540,
      "name": "X",
      "variableType": ""
    },
    {
      "comment": "",
      "id": 1414068576620,
      "name": "Y",
      "variableType": ""
    }
  ]
}