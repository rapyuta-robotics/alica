{
  "blackboard": [
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
      "id": 2850187632373786560,
      "key": "ChooseTestState2BlackboardTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 4512660562305170561,
      "key": "ChooseTestState2MultiPlanInstanceSuccessTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1487451634780526767,
      "key": "ChooseTestState2PlanSuccessTestState",
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
        "x": 250,
        "y": 289
      },
      "state": 4098979167613947533,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 2521443078354411465,
  "inheritBlackboard": false,
  "libraryName": "",
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
        "x": 737.2753842205811,
        "y": 370.44923155883777
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
        "x": 739.1787599282301,
        "y": 593.7031666785609
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
        2841206023261337744,
        4120890224163547783,
        1714211356334603515
      ],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 459,
        "y": 278
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
        "x": 728,
        "y": 110
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "BlackboardTestPlan.pml#4610306063152670816",
          "comment": "",
          "configuration": null,
          "id": 3024879078219750906,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3789769699871163821,
      "inTransitions": [
        1714211356334603515
      ],
      "name": "BlackboardTestState",
      "outTransitions": [],
      "parentPlan": 2521443078354411465,
      "positionWeb": {
        "x": 715.9036833816042,
        "y": -46.231682782429765
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
            "parentKey": "ChooseTestState2BehSuccessTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4487929496627066142,
      "pointsWeb": [],
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
      "pointsWeb": [],
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
      "pointsWeb": [],
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
      "id": 1714211356334603515,
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
      "outState": 3789769699871163821,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2975581900704004835,
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
