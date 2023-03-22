{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 3904653870066436984,
      "key": "y",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 4407030473741374880,
      "key": "x",
      "type": "double"
    }
  ],
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
        "x": 319,
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
        2747861623143659037
      ],
      "parentPlan": 2851720313998235716,
      "positionWeb": {
        "x": 471,
        "y": 316
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
        3302351301741700783
      ],
      "name": "FourCorners",
      "outTransitions": [],
      "parentPlan": 2851720313998235716,
      "positionWeb": {
        "x": 1278.8318424070458,
        "y": 313.8405974920547
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Teleport.beh#4085572422059465423",
          "comment": "",
          "configuration": null,
          "id": 16859304727572262,
          "keyMapping": {
            "input": [
              {
                "childKey": "y",
                "parentKey": "y",
                "value": null
              },
              {
                "childKey": "x",
                "parentKey": "x",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3121720663170820287,
      "inTransitions": [
        3731884057947997339
      ],
      "name": "Teleport",
      "outTransitions": [
        3302351301741700783
      ],
      "parentPlan": 2851720313998235716,
      "positionWeb": {
        "x": 986.154961023046,
        "y": 317.2284616514604
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GenerateRandom2.beh#4020999983789429596",
          "comment": "",
          "configuration": null,
          "id": 2632620596183892341,
          "keyMapping": {
            "input": [
              {
                "childKey": "min",
                "parentKey": null,
                "value": 0.1
              },
              {
                "childKey": "max",
                "parentKey": null,
                "value": 10.0
              }
            ],
            "output": [
              {
                "childKey": "value",
                "parentKey": "y"
              }
            ]
          },
          "name": ""
        },
        {
          "abstractPlan": "GenerateRandom.beh#3356053372497284615",
          "comment": "",
          "configuration": null,
          "id": 3516560261487300370,
          "keyMapping": {
            "input": [
              {
                "childKey": "min",
                "parentKey": null,
                "value": 0.1
              },
              {
                "childKey": "max",
                "parentKey": null,
                "value": 10.0
              }
            ],
            "output": [
              {
                "childKey": "value",
                "parentKey": "x"
              }
            ]
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2594370945855828580,
      "inTransitions": [
        2747861623143659037
      ],
      "name": "GenerateRandomLocation",
      "outTransitions": [
        3731884057947997339
      ],
      "parentPlan": 2851720313998235716,
      "positionWeb": {
        "x": 688.0895614484632,
        "y": 312.3964055888333
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
      "id": 2747861623143659037,
      "inState": 2231154692722562808,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2594370945855828580,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2629062799832896160,
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
      "id": 3302351301741700783,
      "inState": 3121720663170820287,
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
        "id": 662261866873614921,
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
      "id": 3731884057947997339,
      "inState": 2594370945855828580,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3121720663170820287,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2264173628975172003,
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
