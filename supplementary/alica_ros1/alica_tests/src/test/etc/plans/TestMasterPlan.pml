{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 330153981060768900,
      "key": "ChooseTestState2BehSuccessTestState",
      "type": "std::any"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1487451634780526767,
      "key": "ChooseTestState2PlanSuccessTestState",
      "type": "std::any"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 4512660562305170561,
      "key": "ChooseTestState2MultiPlanInstanceSuccessTestState",
      "type": "std::any"
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
      "confAbstractPlanWrappers": [],
      "entryPoint": 3091576485060406140,
      "id": 4098979167613947533,
      "inTransitions": [],
      "name": "ChooseTestState",
      "outTransitions": [
        846865468084822174,
        2841206023261337744,
        4120890224163547783
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
            "parentKey": "ChooseTestState2PlanSuccessTestState"
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
            "parentKey": "ChooseTestState2MultiPlanInstanceSuccessTestState"
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
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
