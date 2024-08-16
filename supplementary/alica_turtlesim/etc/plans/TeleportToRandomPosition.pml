{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 598868303244083964,
      "key": "x",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 2712475924298828232,
      "key": "y",
      "type": "double"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 716543537979003414,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 847199748749956244,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 815756385850055878,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 847199748749956244,
  "implementationName": "TracedPlan",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "TeleportToRandomPosition",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GenerateRandom.beh#3356053372497284615",
          "comment": "",
          "configuration": null,
          "id": 172292232628300347,
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
        },
        {
          "abstractPlan": "GenerateRandom2.beh#4020999983789429596",
          "comment": "",
          "configuration": null,
          "id": 2395328001299149137,
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
        }
      ],
      "entryPoint": 716543537979003414,
      "id": 815756385850055878,
      "inTransitions": [],
      "name": "GenerateRandomX",
      "outTransitions": [
        4247719104545631122
      ],
      "parentPlan": 847199748749956244,
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
          "abstractPlan": "Teleport.beh#4085572422059465423",
          "comment": "",
          "configuration": null,
          "id": 2086117606402602846,
          "keyMapping": {
            "input": [
              {
                "childKey": "x",
                "parentKey": "x",
                "value": null
              },
              {
                "childKey": "y",
                "parentKey": "y",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1435429019417319391,
      "inTransitions": [
        4247719104545631122
      ],
      "name": "Teleport",
      "outTransitions": [
        3964450176861985360
      ],
      "parentPlan": 847199748749956244,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3715359869158732014,
      "inTransitions": [
        3964450176861985360
      ],
      "name": "Success",
      "outTransitions": [],
      "parentPlan": 847199748749956244,
      "positionWeb": {
        "x": 944,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 3964450176861985360,
      "inState": 1435429019417319391,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3715359869158732014,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1374213844849286464,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2",
      "id": 4247719104545631122,
      "inState": 815756385850055878,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1435429019417319391,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3852575510263264987,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
