{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 177437342277134781,
      "isDynamic": false,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "EntryPoint",
      "plan": 3254486013443203397,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 338845808462999166,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3254486013443203397,
  "inheritBlackboard": false,
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
          "keyMapping": {
            "input": [],
            "output": []
          },
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
        "x": 319.0441826215022,
        "y": 308.9558173784978
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
          "keyMapping": {
            "input": [],
            "output": []
          },
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
      "comment": "",
      "condition": "conditions/conditions.cnd#330238006348384830",
      "id": 1914245867924544479,
      "inState": 1114306208475690481,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 338845808462999166,
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
      "condition": "conditions/conditions.cnd#330238006348384830",
      "id": 3345031375302716643,
      "inState": 338845808462999166,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1114306208475690481,
      "pointsWeb": [
        {
          "x": 586,
          "y": 29
        }
      ],
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
