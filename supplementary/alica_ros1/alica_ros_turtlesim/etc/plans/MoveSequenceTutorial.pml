{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3052522711932804870,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1807175145810248577,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1577660109759227973,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 1807175145810248577,
  "implementationName": "TracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": true,
  "name": "MoveSequenceTutorial",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "MoveSequence.pml#2955879459958169673",
          "comment": "",
          "configuration": null,
          "id": 1190477524656036388,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        },
        {
          "abstractPlan": "WaitForTrigger.beh#1320667069122998665",
          "comment": "",
          "configuration": null,
          "id": 3088308470338945807,
          "keyMapping": {
            "input": [
              {
                "childKey": "topic",
                "parentKey": null,
                "value": "rotate"
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2710995558290149564,
      "inTransitions": [
        3233566518007110816,
        1623866678061633764
      ],
      "name": "MoveBetweenCorners",
      "outTransitions": [
        2149585609162989755
      ],
      "parentPlan": 1807175145810248577,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SpawnTurtle.beh#1689864767393644654",
          "comment": "",
          "configuration": null,
          "id": 3944826184966310210,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 3052522711932804870,
      "id": 1577660109759227973,
      "inTransitions": [],
      "name": "SpawnTurtle",
      "outTransitions": [
        3233566518007110816
      ],
      "parentPlan": 1807175145810248577,
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
          "abstractPlan": "RotateTurtle.beh#2948815528667048378",
          "comment": "",
          "configuration": null,
          "id": 3993219790856802160,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        },
        {
          "abstractPlan": "WaitForTrigger.beh#1320667069122998665",
          "comment": "",
          "configuration": null,
          "id": 4301939880152356767,
          "keyMapping": {
            "input": [
              {
                "childKey": "topic",
                "parentKey": null,
                "value": "move"
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2724107460069037337,
      "inTransitions": [
        2149585609162989755
      ],
      "name": "Rotate",
      "outTransitions": [
        1623866678061633764
      ],
      "parentPlan": 1807175145810248577,
      "positionWeb": {
        "x": 944,
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
      "condition": "ConditionRepository.cnd#1",
      "id": 3233566518007110816,
      "inState": 1577660109759227973,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2710995558290149564,
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
        "id": 3981435665385062489,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 2149585609162989755,
      "inState": 2710995558290149564,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2724107460069037337,
      "pointsWeb": [
        {
          "x": 844,
          "y": 29
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 210552878214970537,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 1623866678061633764,
      "inState": 2724107460069037337,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2710995558290149564,
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
        "id": 2903806643656271000,
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
