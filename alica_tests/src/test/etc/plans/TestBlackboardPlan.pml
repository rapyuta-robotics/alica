{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 2314410815911105778,
      "key": "SelectBlackboardTestState2ValueMappingPlanTest",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 2369346233818567219,
      "key": "SelectBlackboardTestState2ValueMappingConditionTest",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3396463320016412716,
      "key": "SelectBlackboardTestState2ValueMappingBehaviourTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 628719349947838626,
      "key": "SelectBlackboardTest2JsonBehaviourKeyMappingTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 2768386542539971136,
      "key": "SelectBlackboardTestState2JsonPlanKeyMappingTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3513844678864778312,
      "key": "SelectBlackboardTestState2JsonBlackboardPlanTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 2553484057661089872,
      "key": "SelectBlackboardTestState2JsonBlackboardBehaviourTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1849994275167166090,
      "key": "SelectBlackboardTestState2NotInheritBlackboardFlagTestPlan",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3900456957389850448,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1001530889734079933,
      "positionWeb": {
        "x": 200,
        "y": 1012
      },
      "state": 2497450171433677722,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1001530889734079933,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "TestBlackboardPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 838628732716716911,
      "inTransitions": [
        4109925817194360283
      ],
      "name": "ValueMappingConditionTestState",
      "outTransitions": [
        656157141551776456
      ],
      "parentPlan": 1001530889734079933,
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
      "id": 2073804979048783675,
      "inTransitions": [
        656157141551776456,
        1606747895267744139,
        1643095952101399822,
        3581537056459357340,
        552536442347424950,
        1151383497195535021,
        3946638270137492226,
        1834379595251689710,
        1144126059138920099
      ],
      "name": "BlackboardTestSuccessState",
      "outTransitions": [],
      "parentPlan": 1001530889734079933,
      "positionWeb": {
        "x": 944,
        "y": 1800
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 3900456957389850448,
      "id": 2497450171433677722,
      "inTransitions": [],
      "name": "SelectBlackboardTestState",
      "outTransitions": [
        210553140637906217,
        4109925817194360283,
        4352638034689014259,
        4467160516593021982,
        287261889354156312,
        1104717051444204592,
        654628426392100100,
        1859443038852090611,
        2126032609564155702
      ],
      "parentPlan": 1001530889734079933,
      "positionWeb": {
        "x": 428,
        "y": 1000
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "ValueMappingTestBeh.beh#1771333734085898056",
          "comment": "",
          "configuration": null,
          "id": 1842914983439127072,
          "keyMapping": {
            "input": [
              {
                "childKey": "inputDouble",
                "parentKey": null,
                "value": 4.5
              },
              {
                "childKey": "inputInt",
                "parentKey": null,
                "value": -9
              },
              {
                "childKey": "inputBool",
                "parentKey": null,
                "value": 1
              },
              {
                "childKey": "inputUint",
                "parentKey": null,
                "value": 5
              },
              {
                "childKey": "inputString",
                "parentKey": null,
                "value": "test"
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3973824651934994954,
      "inTransitions": [
        4352638034689014259
      ],
      "name": "ValueMappingBehaviourTestState",
      "outTransitions": [
        1643095952101399822
      ],
      "parentPlan": 1001530889734079933,
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
          "abstractPlan": "ValueMappingPlansPlan.pml#4191036869656162042",
          "comment": "",
          "configuration": null,
          "id": 2291403747546198247,
          "keyMapping": {
            "input": [
              {
                "childKey": "inputBool",
                "parentKey": null,
                "value": 1
              },
              {
                "childKey": "inputInt",
                "parentKey": null,
                "value": 16
              },
              {
                "childKey": "inputUint",
                "parentKey": null,
                "value": 13
              },
              {
                "childKey": "inputDouble",
                "parentKey": null,
                "value": 0.2
              },
              {
                "childKey": "inputString",
                "parentKey": null,
                "value": "test"
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 4461873388229567864,
      "inTransitions": [
        210553140637906217
      ],
      "name": "ValueMappingPlansTestState",
      "outTransitions": [
        1606747895267744139
      ],
      "parentPlan": 1001530889734079933,
      "positionWeb": {
        "x": 686,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 4162160882769406935,
      "inTransitions": [
        1104717051444204592
      ],
      "name": "NotInheritBlackboardFlagTestPlan",
      "outTransitions": [
        3581537056459357340
      ],
      "parentPlan": 1001530889734079933,
      "positionWeb": {
        "x": 686,
        "y": 1800
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TestPlanKeyMappingPlan.pml#4104108554421232939",
          "comment": "",
          "configuration": null,
          "id": 2166427034405506607,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3904869859094349312,
      "inTransitions": [
        287261889354156312
      ],
      "name": "JsonPlanKeyMappingTestState",
      "outTransitions": [
        1151383497195535021
      ],
      "parentPlan": 1001530889734079933,
      "positionWeb": {
        "x": 686,
        "y": 998.9210573637835
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 2111720692373900619,
      "inTransitions": [
        1859443038852090611
      ],
      "name": "JsonBlackboardBehaviorTestPlan",
      "outTransitions": [
        1144126059138920099
      ],
      "parentPlan": 1001530889734079933,
      "positionWeb": {
        "x": 686,
        "y": 1600
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TestParameterPassingPlan.pml#1179066429431332055",
          "comment": "",
          "configuration": null,
          "id": 1596133500624244980,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 4377107223108143869,
      "inTransitions": [
        654628426392100100
      ],
      "name": "JsonTwoBehaviorKeyMappingTestState",
      "outTransitions": [
        1834379595251689710
      ],
      "parentPlan": 1001530889734079933,
      "positionWeb": {
        "x": 686,
        "y": 800
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3384232317191256948,
      "inTransitions": [
        2126032609564155702
      ],
      "name": "JsonBlackboardPlanTestState",
      "outTransitions": [
        552536442347424950
      ],
      "parentPlan": 1001530889734079933,
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
      "entryPoint": null,
      "id": 185469089366190695,
      "inTransitions": [
        4467160516593021982
      ],
      "name": "JsonBehaviorKeyMappingTestState",
      "outTransitions": [
        3946638270137492226
      ],
      "parentPlan": 1001530889734079933,
      "positionWeb": {
        "x": 686,
        "y": 1200
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
      "id": 210553140637906217,
      "inState": 2497450171433677722,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "SelectBlackboardTestState2ValueMappingPlanTest",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4461873388229567864,
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
        "id": 3897635390720928554,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3799837081018955905",
      "id": 656157141551776456,
      "inState": 838628732716716911,
      "keyMapping": {
        "input": [
          {
            "childKey": "inputString",
            "parentKey": null,
            "value": "test"
          },
          {
            "childKey": "inputInt",
            "parentKey": null,
            "value": 17
          },
          {
            "childKey": "inputDouble",
            "parentKey": null,
            "value": 5.2
          },
          {
            "childKey": "inputBool",
            "parentKey": null,
            "value": 1
          },
          {
            "childKey": "inputUint",
            "parentKey": null,
            "value": 15
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2073804979048783675,
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
        "id": 3992299663798438472,
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
      "id": 1606747895267744139,
      "inState": 4461873388229567864,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2073804979048783675,
      "pointsWeb": [
        {
          "x": 844,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2170219284987848327,
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
      "id": 1643095952101399822,
      "inState": 3973824651934994954,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2073804979048783675,
      "pointsWeb": [
        {
          "x": 844,
          "y": 629
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3511304948193113618,
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
      "id": 4109925817194360283,
      "inState": 2497450171433677722,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "SelectBlackboardTestState2ValueMappingConditionTest",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 838628732716716911,
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
        "id": 4414280795425916603,
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
      "id": 4352638034689014259,
      "inState": 2497450171433677722,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "SelectBlackboardTestState2ValueMappingBehaviourTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3973824651934994954,
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
        "id": 1959687641450480051,
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
      "id": 4467160516593021982,
      "inState": 2497450171433677722,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "SelectBlackboardTest2JsonBehaviourKeyMappingTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 185469089366190695,
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
        "id": 474685147787334296,
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
      "id": 3581537056459357340,
      "inState": 4162160882769406935,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2073804979048783675,
      "pointsWeb": [
        {
          "x": 844,
          "y": 1829
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1582794340363038615,
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
      "id": 552536442347424950,
      "inState": 3384232317191256948,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2073804979048783675,
      "pointsWeb": [
        {
          "x": 844,
          "y": 1429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3729213187630675675,
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
      "id": 287261889354156312,
      "inState": 2497450171433677722,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "SelectBlackboardTestState2JsonPlanKeyMappingTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3904869859094349312,
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
        "id": 2266000768321384035,
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
      "id": 1151383497195535021,
      "inState": 3904869859094349312,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2073804979048783675,
      "pointsWeb": [
        {
          "x": 844,
          "y": 1029
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2492825517458952168,
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
      "id": 1104717051444204592,
      "inState": 2497450171433677722,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "SelectBlackboardTestState2NotInheritBlackboardFlagTestPlan",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4162160882769406935,
      "pointsWeb": [
        {
          "x": 586,
          "y": 1829
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 223642636675583677,
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
      "id": 3946638270137492226,
      "inState": 185469089366190695,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2073804979048783675,
      "pointsWeb": [
        {
          "x": 844,
          "y": 1229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 120766545284196947,
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
      "id": 654628426392100100,
      "inState": 2497450171433677722,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "SelectBlackboardTest2JsonBehaviourKeyMappingTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4377107223108143869,
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
        "id": 3192394300090235196,
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
      "id": 1859443038852090611,
      "inState": 2497450171433677722,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "SelectBlackboardTestState2JsonBlackboardBehaviourTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2111720692373900619,
      "pointsWeb": [
        {
          "x": 586,
          "y": 1629
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4484114288880123826,
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
      "id": 1834379595251689710,
      "inState": 4377107223108143869,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2073804979048783675,
      "pointsWeb": [
        {
          "x": 844,
          "y": 829
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1378553119308395114,
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
      "id": 2126032609564155702,
      "inState": 2497450171433677722,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "SelectBlackboardTestState2JsonBlackboardPlanTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3384232317191256948,
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
        "id": 2122672847962764007,
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
      "id": 1144126059138920099,
      "inState": 2111720692373900619,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2073804979048783675,
      "pointsWeb": [
        {
          "x": 844,
          "y": 1629
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4031292013001563449,
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
