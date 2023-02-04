{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 330153981060768900,
      "key": "ChooseTestState2BehSuccessTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1487451634780526767,
      "key": "ChooseTestState2PlanSuccessTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 4512660562305170561,
      "key": "ChooseTestState2MultiPlanInstanceSuccessTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 2284164419875100184,
      "key": "ChooseTestState2SimpleTestPlanState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1893470949290061309,
      "key": "ChooseTestState2RunBehaviourInSimplePlanState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 4524822962425938639,
      "key": "ChooseTestState2PlanIsSuccessState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 2919359051772364441,
      "key": "ChooseTestState2RuntimeConditionCalledPlanState",
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
  "inheritBlackboard": false,
  "masterPlan": true,
  "name": "TestMasterPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SimpleTestPlan.pml#1412252439925",
          "comment": "",
          "configuration": null,
          "id": 2160624187610871467,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1506398272124884391,
      "inTransitions": [
        2550214909296500141
      ],
      "name": "SimpleTestPlanState",
      "outTransitions": [],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 685.96875,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanIsSuccess.pml#1522377375148",
          "comment": "",
          "configuration": null,
          "id": 3077878256972628511,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1552041115313373349,
      "inTransitions": [
        3219105289339324342
      ],
      "name": "PlanIsSuccess",
      "outTransitions": [],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 685.96875,
        "y": 800
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
        "x": 685.96875,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "RunBehaviourInSimplePlan.pml#2504351804499332310",
          "comment": "",
          "configuration": null,
          "id": 119936314569061232,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3760344851508420724,
      "inTransitions": [
        3312785896666601377
      ],
      "name": "RunBehaviourInSimplePlanState",
      "outTransitions": [],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 685.96875,
        "y": 1000
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
        "x": 685.96875,
        "y": 1200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "RuntimeConditionCalledPlan.pml#3213121947038933654",
          "comment": "",
          "configuration": null,
          "id": 1956335922990661751,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 4045273565408236318,
      "inTransitions": [
        4209515495416542644
      ],
      "name": "RuntimeConditionCalledPlanState",
      "outTransitions": [],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 685.96875,
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
        2550214909296500141,
        2841206023261337744,
        3219105289339324342,
        3312785896666601377,
        4120890224163547783,
        4209515495416542644
      ],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 427.984375,
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
        "x": 685.96875,
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
      "condition": "conditions/ConditionRepository.cnd#3592699233854318376",
      "id": 846865468084822174,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2BehSuccessTestState"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4487929496627066142,
      "pointsWeb": [
        {
          "x": 585.96875,
          "y": 228.9921875
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
      "condition": "conditions/ConditionRepository.cnd#3592699233854318376",
      "id": 2550214909296500141,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2SimpleTestPlanState"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1506398272124884391,
      "pointsWeb": [
        {
          "x": 585.96875,
          "y": 428.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2616157902346364992,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#3592699233854318376",
      "id": 2841206023261337744,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2PlanSuccessTestState"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2212831089687963769,
      "pointsWeb": [
        {
          "x": 585.96875,
          "y": 628.9921875
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
      "condition": "conditions/ConditionRepository.cnd#3592699233854318376",
      "id": 3219105289339324342,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2PlanIsSuccessState"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1552041115313373349,
      "pointsWeb": [
        {
          "x": 585.96875,
          "y": 828.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4584434546591332490,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#3592699233854318376",
      "id": 3312785896666601377,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2RunBehaviourInSimplePlanState"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3760344851508420724,
      "pointsWeb": [
        {
          "x": 585.96875,
          "y": 1028.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4585303539252259897,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#3592699233854318376",
      "id": 4120890224163547783,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2MultiPlanInstanceSuccessTestState"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3960396736820956915,
      "pointsWeb": [
        {
          "x": 585.96875,
          "y": 1228.9921875
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
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#3592699233854318376",
      "id": 4209515495416542644,
      "inState": 4098979167613947533,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2RuntimeConditionCalledPlanState"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4045273565408236318,
      "pointsWeb": [
        {
          "x": 585.96875,
          "y": 1428.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 685495107222979467,
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
