{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1051145281711502661,
      "key": "SuccessOnCondState2WrapperASuccessState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1329563598549023051,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 673160616613514188,
      "positionWeb": {
        "x": 302,
        "y": 333.99999237060547
      },
      "state": 2511956906797886911,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 673160616613514188,
  "implementationName": "",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "SuccessOnCondWrapperAPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 309824621058621265,
      "inTransitions": [
        1123795990168202938
      ],
      "name": "WrapperASuccessState",
      "outTransitions": [],
      "parentPlan": 673160616613514188,
      "positionWeb": {
        "x": 723.078844916524,
        "y": 506.0480382779285
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
          "abstractPlan": "SuccessOnCondPlan.pml#3153116020668535682",
          "comment": "",
          "configuration": null,
          "id": 4450892981403796163,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 1329563598549023051,
      "id": 2511956906797886911,
      "inTransitions": [],
      "name": "SuccessOnCondState",
      "outTransitions": [
        1123795990168202938
      ],
      "parentPlan": 673160616613514188,
      "positionWeb": {
        "x": 491,
        "y": 318.99999237060547
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 1123795990168202938,
      "inState": 2511956906797886911,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "SuccessOnCondState2WrapperASuccessState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 309824621058621265,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4605367163774150375,
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
