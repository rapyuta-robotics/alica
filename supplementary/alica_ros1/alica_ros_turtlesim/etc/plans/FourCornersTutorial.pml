{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 192405225773600915,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2851720313998235716,
      "positionWeb": {
        "x": 151.41046754852027,
        "y": 329
      },
      "state": 2231154692722562808,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 2851720313998235716,
  "implementationName": "TracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": true,
  "name": "FourCornersTutorial",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SpawnTurtle.beh#1689864767393644654",
          "comment": "",
          "configuration": null,
          "id": 1084340226690786551,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 192405225773600915,
      "id": 2231154692722562808,
      "inTransitions": [],
      "name": "SpawnTurtle",
      "outTransitions": [
        264009118846798931
      ],
      "parentPlan": 2851720313998235716,
      "positionWeb": {
        "x": 329.1075291910805,
        "y": 317.1172635496765
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "FourCorners.pml#1225570798912217901",
          "comment": "",
          "configuration": null,
          "id": 1515802129099037226,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3075001531216334549,
      "inTransitions": [
        2987009902483289454
      ],
      "name": "FourCorners",
      "outTransitions": [],
      "parentPlan": 2851720313998235716,
      "positionWeb": {
        "x": 883.0019759405669,
        "y": 320.25457826079355
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TeleportToRandomPosition.pml#847199748749956244",
          "comment": "",
          "configuration": null,
          "id": 3607598770981228246,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 4427016306827673008,
      "inTransitions": [
        264009118846798931
      ],
      "name": "Teleport2RandomPosition",
      "outTransitions": [
        2987009902483289454
      ],
      "parentPlan": 2851720313998235716,
      "positionWeb": {
        "x": 599.6665510852412,
        "y": 316.85289875552866
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
      "id": 2987009902483289454,
      "inState": 4427016306827673008,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3075001531216334549,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1757241392646422745,
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
      "id": 264009118846798931,
      "inState": 2231154692722562808,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4427016306827673008,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2936614415253480616,
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
