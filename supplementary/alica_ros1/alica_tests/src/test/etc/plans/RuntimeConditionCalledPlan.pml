{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 4062982832623374019,
      "key": "runTimeConditionPlanCounter",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3773091174830564135,
      "key": "RuntimeConditionCalledPlanState12RuntimeConditionCalledPlanState2",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1437043949813218310,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3213121947038933654,
      "positionWeb": {
        "x": 377,
        "y": 273
      },
      "state": 2809370282893167096,
      "successRequired": false,
      "task": "taskrepository.tsk#3130906767676893645"
    }
  ],
  "frequency": 0,
  "id": 3213121947038933654,
  "inheritBlackboard": false,
  "masterPlan": false,
  "name": "RuntimeConditionCalledPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": {
    "comment": "",
    "conditionString": "",
    "enabled": false,
    "id": 4595076014383940051,
    "name": "RuntimeConditionPlanCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [],
    "variables": []
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/RuntimeConditionCalledPlanState2Behaviour.beh#704908733785015826",
          "comment": "",
          "configuration": null,
          "id": 556146766096013898,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 991974027058080866,
      "inTransitions": [
        98615919776602683
      ],
      "name": "RuntimeConditionCalledPlanState2",
      "outTransitions": [],
      "parentPlan": 3213121947038933654,
      "positionWeb": {
        "x": 938,
        "y": 299.54562393250865
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/RuntimeConditionCalledPlanState1Behaviour.beh#2580864383983173919",
          "comment": "",
          "configuration": null,
          "id": 1830131175513483682,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 1437043949813218310,
      "id": 2809370282893167096,
      "inTransitions": [],
      "name": "RuntimeConditionCalledPlanState1",
      "outTransitions": [
        98615919776602683
      ],
      "parentPlan": 3213121947038933654,
      "positionWeb": {
        "x": 673,
        "y": 136.818435898763
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
      "id": 98615919776602683,
      "inState": 2809370282893167096,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "RuntimeConditionCalledPlanState12RuntimeConditionCalledPlanState2"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 991974027058080866,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 778972856393262601,
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
