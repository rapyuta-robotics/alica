{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 525979646733597400,
      "key": "ChooseTestState2TransitionAfterInitTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1344107288428655911,
      "key": "ChooseTestState2RepeatedRunCallsTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1409924303843779516,
      "key": "ChooseTestState2PlanAState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3103901468860392175,
      "key": "ChooseTestState2SchedulingPlanTestState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 107070223321998316,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 500355457826444680,
      "positionWeb": {
        "x": 200,
        "y": 612
      },
      "state": 3057463781729421980,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 500355457826444680,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "SchedulingTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TransitionAfterInitTestPlan.pml#523433959791172508",
          "comment": "",
          "configuration": null,
          "id": 450751764880142781,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 59244083871380703,
      "inTransitions": [
        981095176145378775
      ],
      "name": "TransitionAfterInitTestState",
      "outTransitions": [
        1244322502638728653
      ],
      "parentPlan": 500355457826444680,
      "positionWeb": {
        "x": 944,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanB.pml#1629895853508",
          "comment": "",
          "configuration": null,
          "id": 1058623433352532817,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 979397563924779952,
      "inTransitions": [
        3394774278224810473
      ],
      "name": "PlanBState",
      "outTransitions": [
        2802622302473661301,
        3432224151438009037
      ],
      "parentPlan": 500355457826444680,
      "positionWeb": {
        "x": 944,
        "y": 1000
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanA.pml#1629895837159",
          "comment": "",
          "configuration": null,
          "id": 957158909964211446,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1181419849695100093,
      "inTransitions": [
        4587239275124055604
      ],
      "name": "SchedulingPlanTestState",
      "outTransitions": [
        1030521341633603030
      ],
      "parentPlan": 500355457826444680,
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
      "id": 2150587375275705002,
      "inTransitions": [
        1030521341633603030,
        1244322502638728653,
        2802622302473661301,
        3798236463510119931
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 500355457826444680,
      "positionWeb": {
        "x": 1202,
        "y": 1000
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
          "abstractPlan": "PlanA.pml#1629895837159",
          "comment": "",
          "configuration": null,
          "id": 2551833776431600623,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2300166978916418906,
      "inTransitions": [
        3432224151438009037,
        4549972394282787841
      ],
      "name": "PlanAState",
      "outTransitions": [
        3394774278224810473
      ],
      "parentPlan": 500355457826444680,
      "positionWeb": {
        "x": 686,
        "y": 1000
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 107070223321998316,
      "id": 3057463781729421980,
      "inTransitions": [],
      "name": "ChooseTestState",
      "outTransitions": [
        981095176145378775,
        1846034327219291762,
        4549972394282787841,
        4587239275124055604
      ],
      "parentPlan": 500355457826444680,
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
          "abstractPlan": "TestRepeatedRunsBeh.beh#2555589216511791234",
          "comment": "",
          "configuration": null,
          "id": 1603984921708408523,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3830033561903802664,
      "inTransitions": [
        1846034327219291762
      ],
      "name": "RepeatedRunCallsTestState",
      "outTransitions": [
        3798236463510119931
      ],
      "parentPlan": 500355457826444680,
      "positionWeb": {
        "x": 944,
        "y": 600
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
      "id": 981095176145378775,
      "inState": 3057463781729421980,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2TransitionAfterInitTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 59244083871380703,
      "pointsWeb": [
        {
          "x": 586,
          "y": 429
        },
        {
          "x": 715,
          "y": 429
        },
        {
          "x": 844,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 298457776204856019,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2",
      "id": 1030521341633603030,
      "inState": 1181419849695100093,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2150587375275705002,
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
        "id": 4499467508611410270,
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
      "id": 1244322502638728653,
      "inState": 59244083871380703,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2150587375275705002,
      "pointsWeb": [
        {
          "x": 1102,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 131515124962761221,
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
      "id": 1846034327219291762,
      "inState": 3057463781729421980,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2RepeatedRunCallsTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3830033561903802664,
      "pointsWeb": [
        {
          "x": 586,
          "y": 629
        },
        {
          "x": 715,
          "y": 629
        },
        {
          "x": 844,
          "y": 629
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1418256055874275096,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3677391121921226485",
      "id": 2802622302473661301,
      "inState": 979397563924779952,
      "keyMapping": {
        "input": [
          {
            "childKey": "expected",
            "parentKey": null,
            "value": 81
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2150587375275705002,
      "pointsWeb": [
        {
          "x": 1102,
          "y": 1029
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2697492445496840590,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2",
      "id": 3394774278224810473,
      "inState": 2300166978916418906,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 979397563924779952,
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
        "id": 1329224057456655376,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2985374601173748991",
      "id": 3432224151438009037,
      "inState": 979397563924779952,
      "keyMapping": {
        "input": [
          {
            "childKey": "expected",
            "parentKey": null,
            "value": 81
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2300166978916418906,
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
        "id": 926648628100948706,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2",
      "id": 3798236463510119931,
      "inState": 3830033561903802664,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2150587375275705002,
      "pointsWeb": [
        {
          "x": 1102,
          "y": 629
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 638117490984271054,
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
      "id": 4549972394282787841,
      "inState": 3057463781729421980,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2PlanAState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2300166978916418906,
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
        "id": 513814137795501126,
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
      "id": 4587239275124055604,
      "inState": 3057463781729421980,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2SchedulingPlanTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1181419849695100093,
      "pointsWeb": [
        {
          "x": 586,
          "y": 229
        },
        {
          "x": 715,
          "y": 229
        },
        {
          "x": 844,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3424614768652066646,
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
