{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 525979646733597400,
      "key": "ChooseTestState2TransitionAfterInitTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1344107288428655911,
      "key": "ChooseTestState2RepeatedRunCallsTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1375412086750652826,
      "key": "ChooseTestState2ExecOrderTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
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
  "isInterface": false,
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
          "abstractPlan": "SchedulingPlanTestPlan.pml#1655763442597171911",
          "comment": "",
          "configuration": null,
          "id": 2152029821965484833,
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
        3547148819076764273,
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
      "confAbstractPlanWrappers": [],
      "entryPoint": 107070223321998316,
      "id": 3057463781729421980,
      "inTransitions": [],
      "name": "ChooseTestState",
      "outTransitions": [
        981095176145378775,
        1821712422949873757,
        1846034327219291762,
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
          "abstractPlan": "ExecOrderTestPlan.pml#3264434119601111942",
          "comment": "",
          "configuration": null,
          "id": 1157974300791656591,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3067509435331022119,
      "inTransitions": [
        1821712422949873757
      ],
      "name": "ExecOrderTestState",
      "outTransitions": [
        3547148819076764273
      ],
      "parentPlan": 500355457826444680,
      "positionWeb": {
        "x": 751.588502269289,
        "y": 901.7473552663461
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TestRepeatedRunsPlan.pml#1546990961149934975",
          "comment": "",
          "configuration": null,
          "id": 4534718477791868059,
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
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 1821712422949873757,
      "inState": 3057463781729421980,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2ExecOrderTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3067509435331022119,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3398814735947491827,
        "name": "",
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
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2",
      "id": 3547148819076764273,
      "inState": 3067509435331022119,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2150587375275705002,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3979744909541153183,
        "name": "",
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
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
