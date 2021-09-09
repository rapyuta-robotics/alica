{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1402488646221,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1402488634525,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1402488646220,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1402488634525,
  "masterPlan": false,
  "name": "AttackPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Tackle.pml#1402489318663",
          "comment": "",
          "configuration": null,
          "id": 1587718663068,
          "name": "1587718663068"
        },
        {
          "abstractPlan": "behaviours/AttackOpp.beh#1402489351885",
          "comment": "",
          "configuration": null,
          "id": 1587718663070,
          "name": "1587718663070"
        }
      ],
      "entryPoint": 1402488646221,
      "id": 1402488646220,
      "inTransitions": [
        1402489460694
      ],
      "name": "Attack",
      "outTransitions": [
        1402489459382
      ],
      "parentPlan": 1402488634525,
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
          "abstractPlan": "behaviours/Attack.beh#1402488848841",
          "comment": "",
          "configuration": null,
          "id": 1587718663073,
          "name": "1587718663073"
        }
      ],
      "entryPoint": null,
      "id": 1402489396914,
      "inTransitions": [
        1402489459382
      ],
      "name": "Shoot",
      "outTransitions": [
        1402489460694
      ],
      "parentPlan": 1402488634525,
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
      "id": 1402489459382,
      "inState": 1402488646220,
      "name": "MISSING_NAME",
      "outState": 1402489396914,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402489460549,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [
          {
            "comment": "",
            "id": 1403773214317,
            "name": "MISSING_NAME",
            "quantifierType": "all",
            "scope": 1402488634525,
            "sorts": [
              "X",
              "Y"
            ]
          },
          {
            "comment": "",
            "id": 1403773224776,
            "name": "MISSING_NAME",
            "quantifierType": "all",
            "scope": 1402488646220,
            "sorts": [
              "A",
              "B"
            ]
          },
          {
            "comment": "",
            "id": 1403773234841,
            "name": "MISSING_NAME",
            "quantifierType": "all",
            "scope": 1402489396914,
            "sorts": [
              "another one"
            ]
          },
          {
            "comment": "",
            "id": 1403773248357,
            "name": "MISSING_NAME",
            "quantifierType": "all",
            "scope": 1402488646221,
            "sorts": [
              "TaskQuantifier"
            ]
          }
        ],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 1402489460694,
      "inState": 1402489396914,
      "name": "MISSING_NAME",
      "outState": 1402488646220,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "Some nice comment!",
        "enabled": true,
        "id": 1402489462088,
        "name": "ConditionNameShootAttack",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": [
          1403772778288,
          1403772797469,
          1403772816953,
          1403772834750
        ]
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": [
    {
      "comment": "",
      "id": 1403772778288,
      "name": "TestVar1",
      "variableType": "double"
    },
    {
      "comment": "",
      "id": 1403772797469,
      "name": "VarTest2",
      "variableType": "int"
    },
    {
      "comment": "",
      "id": 1403772816953,
      "name": "NewVar",
      "variableType": ""
    },
    {
      "comment": "",
      "id": 1403772834750,
      "name": "ABC",
      "variableType": "FOL"
    }
  ]
}