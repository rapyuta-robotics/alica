{
  "blackboard": [
    {
      "access": "protected",
      "comment": "This is a blackboard entry for testing",
      "id": 3008593874445613740,
      "key": "planOutputKey",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "This is a blackboard entry for testing",
      "id": 4362502621284224779,
      "key": "planInputKey",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "This is a blackboard entry for testing",
      "id": 1779079550934584473,
      "key": "planSecondOutputKey",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "This is a blackboard entry for testing",
      "id": 1907383426694952412,
      "key": "planKey",
      "type": "int64"
    },
    {
      "access": "input",
      "comment": "This is a blackboard entry for testing",
      "id": 785752561217999359,
      "key": "planInputFromMaster",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1610853554686361041,
      "key": "targetChildStatus",
      "type": "std::any"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 58084702421574748,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "ParameterPassingSubplanEP",
      "plan": 1692837668719979457,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1092447442809556626,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1692837668719979457,
  "inheritBlackboard": false,
  "libraryName": "",
  "masterPlan": false,
  "name": "TestParameterPassing",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/TestParameterPassingBehaviour.beh#831400441334251602",
          "comment": "",
          "configuration": null,
          "id": 445396005944825226,
          "keyMapping": {
            "input": [
              {
                "childKey": "behaviorInputKey",
                "parentKey": "planSecondOutputKey",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1529456591400,
      "inTransitions": [
        1129456609900
      ],
      "name": "SecondCall",
      "outTransitions": [
        2229456609900
      ],
      "parentPlan": 1692837668719979457,
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
          "abstractPlan": "behaviours/TestParameterPassingBehaviour.beh#831400441334251602",
          "comment": "",
          "configuration": null,
          "id": 445396005944825225,
          "keyMapping": {
            "input": [
              {
                "childKey": "behaviorInputKey",
                "parentKey": "planOutputKey",
                "value": null
              }
            ],
            "output": [
              {
                "childKey": "behaviorOutputKey",
                "parentKey": "planInputKey"
              }
            ]
          },
          "name": ""
        }
      ],
      "entryPoint": 58084702421574748,
      "id": 1092447442809556626,
      "inTransitions": [
        2229456609900
      ],
      "name": "FirstCall",
      "outTransitions": [
        1129456609900
      ],
      "parentPlan": 1692837668719979457,
      "positionWeb": {
        "x": 357.09572901325475,
        "y": 359.1870397643594
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "Forth",
      "condition": "conditions/ConditionRepository.cnd#843443485857038179",
      "id": 1129456609900,
      "inState": 1092447442809556626,
      "keyMapping": {
        "input": [
          {
            "childKey": "childStatus",
            "parentKey": "targetChildStatus",
            "value": null
          }
        ],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1529456591400,
      "pointsWeb": [
        {
          "x": 586,
          "y": 29
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1529456610600,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "Back",
      "condition": "conditions/ConditionRepository.cnd#1237521027685048666",
      "id": 2229456609900,
      "inState": 1529456591400,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1092447442809556626,
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
        "id": 2529456610600,
        "name": "MISSING_NAME",
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
