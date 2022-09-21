{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 722203880690238135,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 4150733089768927549,
      "positionWeb": {
        "x": 532,
        "y": 409
      },
      "state": 198406198808981916,
      "successRequired": true,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 4150733089768927549,
  "inheritBlackboard": false,
  "masterPlan": true,
  "name": "FailureHandlingMaster",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "FailurePlan.pml#631515556091266493",
          "comment": "",
          "configuration": null,
          "id": 1232744741140546324,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 722203880690238135,
      "id": 198406198808981916,
      "inTransitions": [],
      "name": "FailurePlan",
      "outTransitions": [
        3194919312481305139
      ],
      "parentPlan": 4150733089768927549,
      "positionWeb": {
        "x": 717,
        "y": 397
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 4449850763179483831,
      "inTransitions": [
        3194919312481305139
      ],
      "name": "FailureHandled",
      "outTransitions": [],
      "parentPlan": 4150733089768927549,
      "positionWeb": {
        "x": 925.7482517482517,
        "y": 398.65034965034965
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#190171326790683374",
      "id": 3194919312481305139,
      "inState": 198406198808981916,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4449850763179483831,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 488794245455049811,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
