{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3196801973742034307,
      "key": "targetChildStatus",
      "type": "std::any"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1412252439927,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1412252439927",
      "plan": 1412252439925,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1412252439926,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1412252439925,
  "implementationName": "",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": true,
  "name": "SimpleTestPlan",
  "preCondition": {
    "comment": "",
    "conditionString": "",
    "enabled": true,
    "id": 1412781707952,
    "name": "SimpleTestPlanPreCondition",
    "quantifiers": [],
    "variables": []
  },
  "runtimeCondition": {
    "comment": "",
    "conditionString": "",
    "enabled": false,
    "id": 1412781693884,
    "name": "SimpleTestPlanRuntimeCondition",
    "quantifiers": [],
    "variables": []
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "MidFieldStandard.beh#1402488696205",
          "comment": "",
          "configuration": null,
          "id": 1587718662588,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662588"
        }
      ],
      "entryPoint": 1412252439927,
      "id": 1412252439926,
      "inTransitions": [],
      "name": "TestState1",
      "outTransitions": [
        1412761925032
      ],
      "parentPlan": 1412252439925,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Attack.beh#1402488848841",
          "comment": "",
          "configuration": null,
          "id": 1587718662591,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662591"
        }
      ],
      "entryPoint": null,
      "id": 1412761855746,
      "inTransitions": [
        1412761925032
      ],
      "name": "TestState2",
      "outTransitions": [],
      "parentPlan": 1412252439925,
      "positionWeb": {
        "x": 874.3829160530191,
        "y": 200.69513991163475
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#843443485857038179",
      "id": 1412761925032,
      "inState": 1412252439926,
      "keyMapping": {
        "input": [
          {
            "childKey": "childStatus",
            "parentKey": "targetChildStatus",
            "value": null
          }
        ],
        "output": []
      },
      "name": "1412761925032",
      "outState": 1412761855746,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1412761926856,
        "name": "1412761926856",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": []
}
