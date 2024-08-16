{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 549074179013053940,
      "key": "resultOne",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3537510304698959535,
      "key": "resultTwo",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 4053503531230885495,
      "key": "EntryState2FirstCheckState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 682826694350781678,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1467955996026918944,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 2478721930275583422,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1467955996026918944,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "SameBehInParallelTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 31788376439727489,
      "inTransitions": [
        1251767179803755726
      ],
      "name": "SecondCheckState",
      "outTransitions": [
        4124693711202717699
      ],
      "parentPlan": 1467955996026918944,
      "positionWeb": {
        "x": 944,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 821449835430427634,
      "inTransitions": [
        4124693711202717699
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 1467955996026918944,
      "positionWeb": {
        "x": 1202,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "AddToValueBeh.beh#1629144600464136880",
          "comment": "",
          "configuration": null,
          "id": 3188735900091882738,
          "keyMapping": {
            "input": [
              {
                "childKey": "valueBase",
                "parentKey": null,
                "value": 1
              },
              {
                "childKey": "valueAddition",
                "parentKey": null,
                "value": 2
              }
            ],
            "output": [
              {
                "childKey": "result",
                "parentKey": "resultTwo"
              }
            ]
          },
          "name": ""
        },
        {
          "abstractPlan": "AddToValueBeh.beh#1629144600464136880",
          "comment": "",
          "configuration": null,
          "id": 3609573181032898252,
          "keyMapping": {
            "input": [
              {
                "childKey": "valueBase",
                "parentKey": null,
                "value": 1
              },
              {
                "childKey": "valueAddition",
                "parentKey": null,
                "value": 1
              }
            ],
            "output": [
              {
                "childKey": "result",
                "parentKey": "resultOne"
              }
            ]
          },
          "name": ""
        }
      ],
      "entryPoint": 682826694350781678,
      "id": 2478721930275583422,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [
        4324766460233684331
      ],
      "parentPlan": 1467955996026918944,
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
      "entryPoint": null,
      "id": 4270089538440852737,
      "inTransitions": [
        4324766460233684331
      ],
      "name": "FirstCheckState",
      "outTransitions": [
        1251767179803755726
      ],
      "parentPlan": 1467955996026918944,
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
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 1251767179803755726,
      "inState": 4270089538440852737,
      "keyMapping": {
        "input": [
          {
            "childKey": "left",
            "parentKey": null,
            "value": 2
          },
          {
            "childKey": "right",
            "parentKey": "resultOne",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 31788376439727489,
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
        "id": 1991345008466498526,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 4124693711202717699,
      "inState": 31788376439727489,
      "keyMapping": {
        "input": [
          {
            "childKey": "left",
            "parentKey": null,
            "value": 3
          },
          {
            "childKey": "right",
            "parentKey": "resultTwo",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 821449835430427634,
      "pointsWeb": [
        {
          "x": 1102,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3845638468168372992,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 4324766460233684331,
      "inState": 2478721930275583422,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "EntryState2FirstCheckState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4270089538440852737,
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
        "id": 2652811268841992962,
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
