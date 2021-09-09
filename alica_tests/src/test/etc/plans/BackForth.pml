{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1529456584984,
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
          "abstractPlan": "behaviours/CountIndefinitely.beh#1529456643148",
          "comment": "",
          "configuration": null,
          "id": 1587663024659,
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
          "abstractPlan": "behaviours/CountIndefinitely.beh#1529456643148",
          "comment": "",
          "configuration": null,
          "id": 1587663024662,
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
      "id": 1529456609989,
      "inState": 1529456584983,
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
      "id": 1529456610905,
      "inState": 1529456591410,
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