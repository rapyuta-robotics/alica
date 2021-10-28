{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 177437342277134781,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "",
      "plan": 3254486013443203397,
      "positionWeb": {
        "x": 267,
        "y": 226
      },
      "state": 338845808462999166,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3254486013443203397,
  "masterPlan": true,
  "name": "AdjacentSuccessMasterPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "AdjacentSuccessSubPlan.pml#1682631238618360548",
          "comment": "",
          "configuration": null,
          "id": 1587789133681493604,
          "name": ""
        }
      ],
      "entryPoint": 177437342277134781,
      "id": 338845808462999166,
      "inTransitions": [
        1914245867924544479
      ],
      "name": "EntryState",
      "outTransitions": [
        3345031375302716643
      ],
      "parentPlan": 3254486013443203397,
      "positionWeb": {
        "x": 726,
        "y": 249
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "AdjacentSuccessSubPlan.pml#1682631238618360548",
          "comment": "",
          "configuration": null,
          "id": 1193245213581698231,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1114306208475690481,
      "inTransitions": [
        3345031375302716643
      ],
      "name": "SecondState",
      "outTransitions": [
        1914245867924544479
      ],
      "parentPlan": 3254486013443203397,
      "positionWeb": {
        "x": 1067,
        "y": 276
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "id": 1914245867924544479,
      "inState": 1114306208475690481,
      "name": "",
      "outState": 338845808462999166,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 289358204208851392,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 3345031375302716643,
      "inState": 338845808462999166,
      "name": "",
      "outState": 1114306208475690481,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 807250359520655888,
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