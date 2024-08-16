{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1911668482379994475,
      "key": "ChooseTestState2SamePlanInParallelTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 2582201225876137224,
      "key": "ChooseTestState2SamePlanBehInParallelTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3573177713798947170,
      "key": "ChooseTestState2SameBehInParallelTestState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2953638667385592093,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1081380659938409775,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1345268155455113207,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1081380659938409775,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "SameInParallelTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SamePlanBehInParallelTestPlan.pml#3509966476895996647",
          "comment": "",
          "configuration": null,
          "id": 2336039433650486972,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 31959301521154330,
      "inTransitions": [
        1938510191438787674
      ],
      "name": "SamePlanBehInParallelTestState",
      "outTransitions": [
        4488188169279687143
      ],
      "parentPlan": 1081380659938409775,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 2953638667385592093,
      "id": 1345268155455113207,
      "inTransitions": [],
      "name": "ChooseTestState",
      "outTransitions": [
        1503184172082977465,
        1938510191438787674,
        3952588784917125334
      ],
      "parentPlan": 1081380659938409775,
      "positionWeb": {
        "x": 428,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SamePlanInParallelTestPlan.pml#3704681038071220276",
          "comment": "",
          "configuration": null,
          "id": 4429302951026897325,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3249515609392311493,
      "inTransitions": [
        1503184172082977465
      ],
      "name": "SamePlanInParallelTestState",
      "outTransitions": [
        689497004989039581
      ],
      "parentPlan": 1081380659938409775,
      "positionWeb": {
        "x": 686,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 4141090354361121762,
      "inTransitions": [
        689497004989039581,
        4024840984641598727,
        4488188169279687143
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 1081380659938409775,
      "positionWeb": {
        "x": 1097.7924528301887,
        "y": 401.68867924528297
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SameBehInParallelTestPlan.pml#1467955996026918944",
          "comment": "",
          "configuration": null,
          "id": 3124288014078241123,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 4308812377663194638,
      "inTransitions": [
        3952588784917125334
      ],
      "name": "SameBehInParallelTestState",
      "outTransitions": [
        4024840984641598727
      ],
      "parentPlan": 1081380659938409775,
      "positionWeb": {
        "x": 686,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2",
      "id": 689497004989039581,
      "inState": 3249515609392311493,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4141090354361121762,
      "pointsWeb": [
        {
          "x": 844,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 303284885826278361,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 1503184172082977465,
      "inState": 1345268155455113207,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2SamePlanInParallelTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3249515609392311493,
      "pointsWeb": [
        {
          "x": 586,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3226237409871083787,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 1938510191438787674,
      "inState": 1345268155455113207,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2SamePlanBehInParallelTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 31959301521154330,
      "pointsWeb": [
        {
          "x": 586,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3286380994036243446,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 3952588784917125334,
      "inState": 1345268155455113207,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2SameBehInParallelTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4308812377663194638,
      "pointsWeb": [
        {
          "x": 586,
          "y": 629
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 929945440347983592,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2",
      "id": 4024840984641598727,
      "inState": 4308812377663194638,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4141090354361121762,
      "pointsWeb": [
        {
          "x": 844,
          "y": 629
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 642054276601673017,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2",
      "id": 4488188169279687143,
      "inState": 31959301521154330,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4141090354361121762,
      "pointsWeb": [
        {
          "x": 844,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2305170794111983802,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
