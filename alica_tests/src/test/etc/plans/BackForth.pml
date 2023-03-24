{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1529456584984,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1529456584982,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1529456584983,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1529456584982,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": true,
  "name": "BackForth",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "CountIndefinitely.beh#1529456643148",
          "comment": "",
          "configuration": null,
          "id": 1587663024659,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587663024659"
        }
      ],
      "entryPoint": 1529456584984,
      "id": 1529456584983,
      "inTransitions": [
        1529456610905
      ],
      "name": "First",
      "outTransitions": [
        1529456609989
      ],
      "parentPlan": 1529456584982,
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
          "abstractPlan": "CountIndefinitely.beh#1529456643148",
          "comment": "",
          "configuration": null,
          "id": 1587663024662,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587663024662"
        }
      ],
      "entryPoint": null,
      "id": 1529456591410,
      "inTransitions": [
        1529456609989
      ],
      "name": "Second",
      "outTransitions": [
        1529456610905
      ],
      "parentPlan": 1529456584982,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "Forth",
      "condition": "ConditionRepository.cnd#3016035752801585170",
      "id": 1529456609989,
      "inState": 1529456584983,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1529456591410,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1529456610697,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "Back",
      "condition": "ConditionRepository.cnd#1556522827919252115",
      "id": 1529456610905,
      "inState": 1529456591410,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1529456584983,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1529456611916,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": []
}
