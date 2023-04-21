{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 295638103823734752,
      "key": "ChooseTestState2AdjacentSuccessTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 330153981060768900,
      "key": "ChooseTestState2BehSuccessTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1487451634780526767,
      "key": "ChooseTestState2PlanSuccessTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 2321498751406550734,
      "key": "ChooseTestState2BlackboardTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3151624094246832948,
      "key": "ChooseTestState2StandardLibraryCompareConditionsState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3421733107371844672,
      "key": "ChooseTestState2IsChildSuccessTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 4512660562305170561,
      "key": "ChooseTestState2MultiPlanInstanceSuccessTestState",
      "type": "bool"
    }
  ],
  "comment": "The master plan of the alica tests. It is used to choose which test to execute",
  "entryPoints": [
    {
      "comment": "",
      "id": 3091576485060406140,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2521443078354411465,
      "positionWeb": {
        "x": 200,
        "y": 812
      },
      "state": 4098979167613947533,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 2521443078354411465,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": true,
  "name": "TestMasterPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "IsChildSuccessTestPlan.pml#2027765331130289429",
          "comment": "",
          "configuration": null,
          "id": 2430429930355032718,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 235414165593595069,
      "inTransitions": [
        3651332618963874336
      ],
      "name": "IsChildSuccessTestState",
      "outTransitions": [],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 686,
        "y": 1000
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "AdjacentSuccessTestPlan.pml#2039053377176713134",
          "comment": "",
          "configuration": null,
          "id": 2655529348250165827,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1016088688698752054,
      "inTransitions": [
        3815101160319869321
      ],
      "name": "AdjacentSuccessTestState",
      "outTransitions": [],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 686,
        "y": 1200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "BlackboardTestPlan.pml#1633245244310547016",
          "comment": "",
          "configuration": null,
          "id": 1158711423955439419,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2132957861522514570,
      "inTransitions": [
        978339276131155715
      ],
      "name": "BlackboardTestState",
      "outTransitions": [],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 686,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanSuccessTestPlan.pml#3870436056558842479",
          "comment": "",
          "configuration": null,
          "id": 2048990716663744774,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2212831089687963769,
      "inTransitions": [
        2841206023261337744
      ],
      "name": "PlanSuccessTestState",
      "outTransitions": [],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 686,
        "y": 800
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "StandardLibraryCompareConditionsPlan.pml#570949913365648849",
          "comment": "",
          "configuration": null,
          "id": 3326872204798256197,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3029581745457050844,
      "inTransitions": [
        1234148291059202190
      ],
      "name": "StandardLibraryCompareConditionsState",
      "outTransitions": [],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 686,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "MultiPlanInstanceSuccessTestPlan.pml#3392981108193862307",
          "comment": "",
          "configuration": null,
          "id": 2887093750412106903,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3960396736820956915,
      "inTransitions": [
        4120890224163547783
      ],
      "name": "MultiPlanInstanceSuccessTestState",
      "outTransitions": [],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 686,
        "y": 1400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 3091576485060406140,
      "id": 4098979167613947533,
      "inTransitions": [],
      "name": "ChooseTestState",
      "outTransitions": [
        846865468084822174,
        978339276131155715,
        1234148291059202190,
        2841206023261337744,
        3651332618963874336,
        3815101160319869321,
        4120890224163547783
      ],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 428,
        "y": 800
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "BehSuccessTestPlan.pml#2189867578804904568",
          "comment": "",
          "configuration": null,
          "id": 541298159829395728,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 4487929496627066142,
      "inTransitions": [
        846865468084822174
      ],
      "name": "BehSuccessTestState",
      "outTransitions": [],
      "parentPlan": 2521443078354411465,
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
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 846865468084822174,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2BehSuccessTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4487929496627066142,
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
        "id": 1879497210052616817,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 978339276131155715,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2BlackboardTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2132957861522514570,
      "pointsWeb": [
        {
          "x": 586,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3320726945359323759,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 1234148291059202190,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2StandardLibraryCompareConditionsState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3029581745457050844,
      "pointsWeb": [
        {
          "x": 586,
          "y": 629
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4560357964186391260,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 2841206023261337744,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2PlanSuccessTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2212831089687963769,
      "pointsWeb": [
        {
          "x": 586,
          "y": 829
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3883605426713053219,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 3651332618963874336,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2IsChildSuccessTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 235414165593595069,
      "pointsWeb": [
        {
          "x": 586,
          "y": 1029
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1434924428337492941,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 3815101160319869321,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2AdjacentSuccessTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1016088688698752054,
      "pointsWeb": [
        {
          "x": 586,
          "y": 1229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2822951189378292075,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 4120890224163547783,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2MultiPlanInstanceSuccessTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3960396736820956915,
      "pointsWeb": [
        {
          "x": 586,
          "y": 1429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2733591692277574870,
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
