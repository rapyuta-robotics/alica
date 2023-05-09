{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 1409924303843779516,
      "key": "ChooseTestState2OrderedRunTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3103901468860392175,
      "key": "ChooseTestState2SchedulingPlanInitTermTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1344107288428655911,
      "key": "ChooseTestState2BehaviourRunSchedulingTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 525979646733597400,
      "key": "ChooseTestState2ExecuteBehaviourTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1272213235792880477,
      "key": "ChooseTestState2OrderedSchedulingTestState",
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
          "abstractPlan": "BehaviourRunSchedulingTestPlan.pml#3688442640782714083",
          "comment": "",
          "configuration": null,
          "id": 575057991463670178,
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
        4549972394282787841
      ],
      "name": "BehaviourRunSchedulingTestState",
      "outTransitions": [
        2186958297636364279
      ],
      "parentPlan": 500355457826444680,
      "positionWeb": {
        "x": 944,
        "y": 800
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SchedulingPlanInitTermTestPlan.pml#3592474165381227575",
          "comment": "",
          "configuration": null,
          "id": 893199869486114622,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2973240550467190618,
      "inTransitions": [
        1670278943433390378
      ],
      "name": "SchedulingPlanInitTermTestState",
      "outTransitions": [
        389440402862114838
      ],
      "parentPlan": 500355457826444680,
      "positionWeb": {
        "x": 686,
        "y": 600
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
        4587239275124055604,
        1846034327219291762,
        1670278943433390378,
        4549972394282787841
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
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 504810086849143784,
      "inTransitions": [
        389440402862114838
      ],
      "name": "TerminateTestPlans",
      "outTransitions": [
        1194731986936733246
      ],
      "parentPlan": 500355457826444680,
      "positionWeb": {
        "x": 944,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "OrderedSchedulingTestPlan.pml#1629895582410",
          "comment": "",
          "configuration": null,
          "id": 2679216822667336369,
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
      "name": "OrderedSchedulingTestState",
      "outTransitions": [
        1030521341633603030
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
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 2150587375275705002,
      "inTransitions": [
        3798236463510119931,
        1244322502638728653,
        2186958297636364279,
        1030521341633603030,
        1194731986936733246
      ],
      "name": "",
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
          "abstractPlan": "ExecuteBehaviourTestPlan.pml#3848527741370035673",
          "comment": "",
          "configuration": null,
          "id": 3943338218099735211,
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
      "name": "ExecuteBehaviourTestState",
      "outTransitions": [
        3798236463510119931
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
          "abstractPlan": "OrderedRunTestPlan.pml#3497513845787611552",
          "comment": "",
          "configuration": null,
          "id": 4185803220880555955,
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
      "name": "OrderedRunTestState",
      "outTransitions": [
        1244322502638728653
      ],
      "parentPlan": 500355457826444680,
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
          "y": 1029
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
      "condition": "ConditionRepository.cnd#2",
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
          "y": 229
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
      "id": 981095176145378775,
      "inState": 3057463781729421980,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2OrderedRunTestState",
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
      "id": 389440402862114838,
      "inState": 2973240550467190618,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 504810086849143784,
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
        "id": 3762716398681025427,
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
      "id": 2186958297636364279,
      "inState": 2300166978916418906,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2150587375275705002,
      "pointsWeb": [
        {
          "x": 1102,
          "y": 829
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3652564732974074158,
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
          "y": 429
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
      "condition": "ConditionRepository.cnd#4292488527327751514",
      "id": 1194731986936733246,
      "inState": 504810086849143784,
      "keyMapping": {
        "input": [
          {
            "childKey": "value",
            "parentKey": null,
            "value": 6
          }
        ],
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
        "id": 1024311282442391850,
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
            "parentKey": "ChooseTestState2OrderedSchedulingTestState",
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
        "id": 3424614768652066646,
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
            "parentKey": "ChooseTestState2ExecuteBehaviourTestState",
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
          "y": 1029
        },
        {
          "x": 715,
          "y": 1029
        },
        {
          "x": 844,
          "y": 1029
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
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 1670278943433390378,
      "inState": 3057463781729421980,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2SchedulingPlanInitTermTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2973240550467190618,
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
        "id": 1414591043954276742,
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
            "parentKey": "ChooseTestState2BehaviourRunSchedulingTestState",
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
          "y": 829
        },
        {
          "x": 715,
          "y": 829
        },
        {
          "x": 844,
          "y": 829
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
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
