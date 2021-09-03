{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1418825395941,
      "maxCardinality": 10000,
      "minCardinality": 1,
      "name": "AttackTask",
      "plan": 1418825395939,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1418825395940,
      "successRequired": false,
      "task": "taskrepository.tsk#1407153522080"
    },
    {
      "comment": "",
      "id": 1418825402617,
      "maxCardinality": 10000,
      "minCardinality": 1,
      "name": "DefaultTask",
      "plan": 1418825395939,
      "positionWeb": {
        "x": 200,
        "y": 612
      },
      "state": 1418825404963,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1418825395939,
  "masterPlan": false,
  "name": "MasterSyncTransition",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1418825395941,
      "id": 1418825395940,
      "inTransitions": [],
      "name": "FirstTaskFirstState",
      "outTransitions": [
        1418825425833
      ],
      "parentPlan": 1418825395939,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1418825402617,
      "id": 1418825404963,
      "inTransitions": [],
      "name": "SecondTaskFirstState",
      "outTransitions": [
        1418825427469
      ],
      "parentPlan": 1418825395939,
      "positionWeb": {
        "x": 428,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/Attack.beh#1402488848841",
          "comment": "",
          "configuration": null,
          "id": 1587718662770,
          "name": "1587718662770"
        }
      ],
      "entryPoint": null,
      "id": 1418825409988,
      "inTransitions": [
        1418825425833
      ],
      "name": "FirstTaskSecondState",
      "outTransitions": [],
      "parentPlan": 1418825395939,
      "positionWeb": {
        "x": 918,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/Attack.beh#1402488848841",
          "comment": "",
          "configuration": null,
          "id": 1587718662773,
          "name": "1587718662773"
        }
      ],
      "entryPoint": null,
      "id": 1418825411686,
      "inTransitions": [
        1418825427469
      ],
      "name": "SecondTaskSecondState",
      "outTransitions": [],
      "parentPlan": 1418825395939,
      "positionWeb": {
        "x": 918,
        "y": 800
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [
    {
      "comment": "",
      "failOnSyncTimeout": false,
      "id": 1418825482116,
      "name": "Sync",
      "plan": 1418825395939,
      "positionWeb": {
        "x": 435,
        "y": 807
      },
      "syncTimeout": 10000,
      "syncedTransitions": [
        1418825425833,
        1418825427469
      ],
      "talkTimeout": 30
    }
  ],
  "transitions": [
    {
      "comment": "",
      "id": 1418825425833,
      "inState": 1418825395940,
      "name": "FirstTaskTran",
      "outState": 1418825409988,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1418825427317,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": 1418825482116
    },
    {
      "comment": "",
      "id": 1418825427469,
      "inState": 1418825404963,
      "name": "SecondTaskTran",
      "outState": 1418825411686,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1418825428924,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": 1418825482116
    }
  ],
  "utilityThreshold": 0.1,
  "variables": []
}