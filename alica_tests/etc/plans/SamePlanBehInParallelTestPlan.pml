{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 762584330532259947,
      "key": "planResultTwo",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 772996229802355990,
      "key": "behResultOne",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 845172727750198208,
      "key": "behResultTwo",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 4501937621687609404,
      "key": "planResultOne",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 4576317543403705127,
      "key": "EntryState2FirstCheckState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 691518218277769656,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3509966476895996647,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 518610467558101325,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3509966476895996647,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "SamePlanBehInParallelTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "AddToValuePlan.pml#1897460300157750096",
          "comment": "",
          "configuration": null,
          "id": 88232721083847073,
          "keyMapping": {
            "input": [
              {
                "childKey": "valueBase",
                "parentKey": null,
                "value": 3
              },
              {
                "childKey": "valueAddition",
                "parentKey": null,
                "value": 4
              }
            ],
            "output": [
              {
                "childKey": "result",
                "parentKey": "planResultOne"
              }
            ]
          },
          "name": ""
        },
        {
          "abstractPlan": "AddToValueBeh.beh#1629144600464136880",
          "comment": "",
          "configuration": null,
          "id": 2820359424214552703,
          "keyMapping": {
            "input": [
              {
                "childKey": "valueBase",
                "parentKey": null,
                "value": 3
              },
              {
                "childKey": "valueAddition",
                "parentKey": null,
                "value": 5
              }
            ],
            "output": [
              {
                "childKey": "result",
                "parentKey": "behResultTwo"
              }
            ]
          },
          "name": ""
        },
        {
          "abstractPlan": "AddToValueBeh.beh#1629144600464136880",
          "comment": "",
          "configuration": null,
          "id": 3397201732500445118,
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
                "parentKey": "behResultOne"
              }
            ]
          },
          "name": ""
        },
        {
          "abstractPlan": "AddToValuePlan.pml#1897460300157750096",
          "comment": "",
          "configuration": null,
          "id": 3923573132730566752,
          "keyMapping": {
            "input": [
              {
                "childKey": "valueAddition",
                "parentKey": null,
                "value": 3
              },
              {
                "childKey": "valueBase",
                "parentKey": null,
                "value": 15
              }
            ],
            "output": [
              {
                "childKey": "result",
                "parentKey": "planResultTwo"
              }
            ]
          },
          "name": ""
        }
      ],
      "entryPoint": 691518218277769656,
      "id": 518610467558101325,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [
        1387122566800131754
      ],
      "parentPlan": 3509966476895996647,
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
      "id": 972722523665333575,
      "inTransitions": [
        606098706621144239
      ],
      "name": "FourthCheckState",
      "outTransitions": [
        71477950978277121
      ],
      "parentPlan": 3509966476895996647,
      "positionWeb": {
        "x": 1460,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1556255341256380945,
      "inTransitions": [
        1387122566800131754
      ],
      "name": "FirstCheckState",
      "outTransitions": [
        3146491180749929825
      ],
      "parentPlan": 3509966476895996647,
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
      "id": 1734405499683597796,
      "inTransitions": [
        3146491180749929825
      ],
      "name": "SecondCheckState",
      "outTransitions": [
        2836862209587330715
      ],
      "parentPlan": 3509966476895996647,
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
      "id": 2183675051573163767,
      "inTransitions": [
        2836862209587330715
      ],
      "name": "ThirdCheckState",
      "outTransitions": [
        606098706621144239
      ],
      "parentPlan": 3509966476895996647,
      "positionWeb": {
        "x": 1202,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3958068997518267489,
      "inTransitions": [
        71477950978277121
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 3509966476895996647,
      "positionWeb": {
        "x": 1718,
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
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 71477950978277121,
      "inState": 972722523665333575,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "behResultTwo",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 8
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3958068997518267489,
      "pointsWeb": [
        {
          "x": 1618,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2558319661179124456,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 606098706621144239,
      "inState": 2183675051573163767,
      "keyMapping": {
        "input": [
          {
            "childKey": "left",
            "parentKey": null,
            "value": 3
          },
          {
            "childKey": "right",
            "parentKey": "behResultOne",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 972722523665333575,
      "pointsWeb": [
        {
          "x": 1360,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2690242046097364267,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 1387122566800131754,
      "inState": 518610467558101325,
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
      "outState": 1556255341256380945,
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
        "id": 4516231747657072754,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 2836862209587330715,
      "inState": 1734405499683597796,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "planResultTwo",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 18
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2183675051573163767,
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
        "id": 4449607745772643430,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 3146491180749929825,
      "inState": 1556255341256380945,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "planResultOne",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 7
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1734405499683597796,
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
        "id": 2724717422267100016,
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
