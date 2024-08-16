{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 462984650222620929,
      "key": "resultOne",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 2682650488249660408,
      "key": "resultTwo",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3145297303645115724,
      "key": "EntryState2FirstCheckState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3615653659634051508,
      "key": "resultThree",
      "type": "int64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3073827618371975266,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3704681038071220276,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 871981763418632836,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3704681038071220276,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "SamePlanInParallelTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 20390315026263146,
      "inTransitions": [
        443562486491116060
      ],
      "name": "ThirdCheckState",
      "outTransitions": [
        1284767508458725786
      ],
      "parentPlan": 3704681038071220276,
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
      "id": 584493314305051000,
      "inTransitions": [
        1284767508458725786
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 3704681038071220276,
      "positionWeb": {
        "x": 1460,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 633860451304356972,
      "inTransitions": [
        1439053087362562600
      ],
      "name": "FirstCheckState",
      "outTransitions": [
        3364382605518057894
      ],
      "parentPlan": 3704681038071220276,
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
          "abstractPlan": "AddToValuePlan.pml#1897460300157750096",
          "comment": "",
          "configuration": null,
          "id": 2486062502802677337,
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
          "abstractPlan": "AddToValuePlan.pml#1897460300157750096",
          "comment": "",
          "configuration": null,
          "id": 2571396190658341401,
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
        },
        {
          "abstractPlan": "AddToValuePlan.pml#1897460300157750096",
          "comment": "",
          "configuration": null,
          "id": 3600004838610145670,
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
                "value": 1
              }
            ],
            "output": [
              {
                "childKey": "result",
                "parentKey": "resultThree"
              }
            ]
          },
          "name": ""
        }
      ],
      "entryPoint": 3073827618371975266,
      "id": 871981763418632836,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [
        1439053087362562600
      ],
      "parentPlan": 3704681038071220276,
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
      "id": 3639860252154425997,
      "inTransitions": [
        3364382605518057894
      ],
      "name": "SecondCheckState",
      "outTransitions": [
        443562486491116060
      ],
      "parentPlan": 3704681038071220276,
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
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 443562486491116060,
      "inState": 3639860252154425997,
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
      "outState": 20390315026263146,
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
        "id": 1582096676736674571,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 1284767508458725786,
      "inState": 20390315026263146,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "resultThree",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 4
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 584493314305051000,
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
        "id": 3107594594218200822,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 1439053087362562600,
      "inState": 871981763418632836,
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
      "outState": 633860451304356972,
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
        "id": 2412273558973067821,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 3364382605518057894,
      "inState": 633860451304356972,
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
      "outState": 3639860252154425997,
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
        "id": 355432440240654344,
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
