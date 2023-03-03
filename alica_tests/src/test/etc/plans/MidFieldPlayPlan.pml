{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1402488787819,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1402488770050,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1402488787818,
      "successRequired": true,
      "task": "taskrepository.tsk#1225112227903"
    },
    {
      "comment": "",
      "id": 1402500828244,
      "isDynamic": false,
      "maxCardinality": 5,
      "minCardinality": 3,
      "name": "NewEntryPoint",
      "plan": 1402488770050,
      "positionWeb": {
        "x": 200,
        "y": 812
      },
      "state": 1402500830885,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1402488770050,
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "MidFieldPlayPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": {
    "comment": "",
    "conditionString": "",
    "enabled": false,
    "id": 1402489260911,
    "name": "MidFieldPlayPlanRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [],
    "variables": []
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/MidFieldStandard.beh#1402488696205",
          "comment": "",
          "configuration": null,
          "id": 1587718662682,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662682"
        }
      ],
      "entryPoint": 1402488787819,
      "id": 1402488787818,
      "inTransitions": [],
      "name": "Wander",
      "outTransitions": [
        1402489257607,
        1402489276995
      ],
      "parentPlan": 1402488770050,
      "positionWeb": {
        "x": 428,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Tackle.pml#1402489318663",
          "comment": "",
          "configuration": null,
          "id": 1587718662685,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662685"
        },
        {
          "abstractPlan": "behaviours/Attack.beh#1402488848841",
          "comment": "",
          "configuration": null,
          "id": 1587718662687,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662687"
        },
        {
          "abstractPlan": "Defend.pml#1402488893641",
          "comment": "",
          "configuration": null,
          "id": 1587718662689,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662689"
        }
      ],
      "entryPoint": null,
      "id": 1402489237914,
      "inTransitions": [
        1402489257607
      ],
      "name": "Tackle",
      "outTransitions": [],
      "parentPlan": 1402488770050,
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
          "abstractPlan": "behaviours/Tackle.beh#1402488939130",
          "comment": "",
          "configuration": null,
          "id": 1587718662692,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718662692"
        }
      ],
      "entryPoint": null,
      "id": 1402489273401,
      "inTransitions": [
        1402489276995
      ],
      "name": "Sync",
      "outTransitions": [],
      "parentPlan": 1402488770050,
      "positionWeb": {
        "x": 944,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1402500828244,
      "id": 1402500830885,
      "inTransitions": [],
      "name": "Kill",
      "outTransitions": [
        1402500843072
      ],
      "parentPlan": 1402488770050,
      "positionWeb": {
        "x": 428,
        "y": 800
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1402500833246,
      "inTransitions": [
        1402500843072
      ],
      "name": "Shoot",
      "outTransitions": [],
      "parentPlan": 1402488770050,
      "positionWeb": {
        "x": 944,
        "y": 1000
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [
    {
      "comment": "",
      "failOnSyncTimeout": false,
      "id": 1402500865502,
      "name": "SynChro",
      "plan": 1402488770050,
      "positionWeb": {
        "x": 435,
        "y": 1007
      },
      "syncTimeout": 10000,
      "syncedTransitions": [
        1402489276995,
        1402500843072
      ],
      "talkTimeout": 30
    }
  ],
  "transitions": [
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#1678986049909129132",
      "id": 1402489257607,
      "inState": 1402488787818,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1402489237914,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402489258509,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#1678986049909129132",
      "id": 1402489276995,
      "inState": 1402488787818,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1402489273401,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402489278408,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": 1402500865502
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#1678986049909129132",
      "id": 1402500843072,
      "inState": 1402500830885,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1402500833246,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402500844446,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": 1402500865502
    }
  ],
  "utilityThreshold": 0.1,
  "variables": []
}
