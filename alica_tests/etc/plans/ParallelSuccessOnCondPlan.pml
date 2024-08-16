{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3797646244448202675,
      "key": "WaitForTriggerState2ParallelExecState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1461040637399587631,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3288843407985944525,
      "positionWeb": {
        "x": 343,
        "y": 302.99999237060547
      },
      "state": 1899337663064771436,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3288843407985944525,
  "implementationName": "",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "ParallelSuccessOnCondPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SuccessOnCondWrapperBPlan.pml#2869465844414224272",
          "comment": "",
          "configuration": null,
          "id": 1533990533001566061,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        },
        {
          "abstractPlan": "SuccessOnCondWrapperAPlan.pml#673160616613514188",
          "comment": "",
          "configuration": null,
          "id": 2172941199313750227,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1413250009777584468,
      "inTransitions": [
        4409408749212342883
      ],
      "name": "ParallelExecState",
      "outTransitions": [
        602472383731208509
      ],
      "parentPlan": 3288843407985944525,
      "positionWeb": {
        "x": 757,
        "y": 435.99999237060547
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1461040637399587631,
      "id": 1899337663064771436,
      "inTransitions": [],
      "name": "WaitForTriggerState",
      "outTransitions": [
        4409408749212342883
      ],
      "parentPlan": 3288843407985944525,
      "positionWeb": {
        "x": 512,
        "y": 288.99999237060547
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 4136091023931334823,
      "inTransitions": [
        602472383731208509
      ],
      "name": "ParallelPlanSuccessState",
      "outTransitions": [],
      "parentPlan": 3288843407985944525,
      "positionWeb": {
        "x": 986,
        "y": 294.99999237060547
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
      "condition": "ConditionRepository.cnd#2",
      "id": 602472383731208509,
      "inState": 1413250009777584468,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4136091023931334823,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2767999024419231358,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 4409408749212342883,
      "inState": 1899337663064771436,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "WaitForTriggerState2ParallelExecState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1413250009777584468,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1470823850869867131,
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
