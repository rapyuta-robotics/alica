{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1223260418939648240,
      "key": "WaitForCondState2CondSuccessState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 173667041779969680,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3153116020668535682,
      "positionWeb": {
        "x": 354,
        "y": 277.99999237060547
      },
      "state": 4012637487828402178,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3153116020668535682,
  "implementationName": "",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "SuccessOnCondPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 327448157837662747,
      "inTransitions": [
        3580106698787080417
      ],
      "name": "CondSuccessState",
      "outTransitions": [],
      "parentPlan": 3153116020668535682,
      "positionWeb": {
        "x": 833,
        "y": 369.99999237060547
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 173667041779969680,
      "id": 4012637487828402178,
      "inTransitions": [],
      "name": "WaitForCondState",
      "outTransitions": [
        3580106698787080417
      ],
      "parentPlan": 3153116020668535682,
      "positionWeb": {
        "x": 547,
        "y": 264.99999237060547
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
      "id": 3580106698787080417,
      "inState": 4012637487828402178,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "WaitForCondState2CondSuccessState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 327448157837662747,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 92747471708069515,
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
