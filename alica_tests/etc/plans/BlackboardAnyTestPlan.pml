{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 596717002024153194,
      "key": "ChooseTestState2OverwriteValueWithDifferentTypeTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3183790171424958342,
      "key": "ChooseTestState2SetAsAnyTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1468721377253480800,
      "key": "ChooseTestState2AccessAsAnyTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3713318294945971428,
      "key": "ChooseTestState2SetWithSpecifyingTypeTestState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3051358524960609998,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1578937713657933189,
      "positionWeb": {
        "x": 190,
        "y": 231.578125
      },
      "state": 71270235805491939,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1578937713657933189,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "BlackboardAnyTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "OverwriteValueWithDifferentTypeTestBeh.beh#4399049787950021473",
          "comment": "",
          "configuration": null,
          "id": 915959907932490882,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 454026604792850007,
      "inTransitions": [
        3030470219591531308
      ],
      "name": "OverwriteValueWIthDifferentTypeTestState",
      "outTransitions": [
        911501001947067876
      ],
      "parentPlan": 1578937713657933189,
      "positionWeb": {
        "x": 702,
        "y": 131.578125
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3534693618802586510,
      "inTransitions": [
        3452290056321843317,
        911501001947067876,
        3737969493616125732,
        3923063415261756327
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 1578937713657933189,
      "positionWeb": {
        "x": 1117,
        "y": 352.578125
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
          "abstractPlan": "SetAsAnyTestBeh.beh#215712651194641312",
          "comment": "",
          "configuration": null,
          "id": 1434558316339666457,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2292066748998403089,
      "inTransitions": [
        1166208708564119385
      ],
      "name": "SetAsAnyTestState",
      "outTransitions": [
        3452290056321843317
      ],
      "parentPlan": 1578937713657933189,
      "positionWeb": {
        "x": 746,
        "y": 449.578125
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "AccessAsAnyTestBeh.beh#103008403659174438",
          "comment": "",
          "configuration": null,
          "id": 3105622622258023190,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 4042038736016474424,
      "inTransitions": [
        3285909339681176986
      ],
      "name": "AccessAsAnyTestState",
      "outTransitions": [
        3737969493616125732
      ],
      "parentPlan": 1578937713657933189,
      "positionWeb": {
        "x": 764.8566433566433,
        "y": 745.9460773601397
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SetWithSpecifyingTypeTestBeh.beh#1967012207662912929",
          "comment": "",
          "configuration": null,
          "id": 1232792656047449218,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2489491116942426158,
      "inTransitions": [
        3920671252638360136
      ],
      "name": "SetWithSpecifyingTypeTestState",
      "outTransitions": [
        3923063415261756327
      ],
      "parentPlan": 1578937713657933189,
      "positionWeb": {
        "x": 751.865034965035,
        "y": 608.6062172202796
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 3051358524960609998,
      "id": 71270235805491939,
      "inTransitions": [],
      "name": "ChooseTestState",
      "outTransitions": [
        1166208708564119385,
        3920671252638360136,
        3030470219591531308,
        3285909339681176986
      ],
      "parentPlan": 1578937713657933189,
      "positionWeb": {
        "x": 440,
        "y": 247.578125
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 3452290056321843317,
      "inState": 2292066748998403089,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3534693618802586510,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2279792593028974864,
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
      "id": 911501001947067876,
      "inState": 454026604792850007,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3534693618802586510,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4444247153084361728,
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
      "id": 1166208708564119385,
      "inState": 71270235805491939,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2SetAsAnyTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2292066748998403089,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1223241439336082757,
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
      "id": 3920671252638360136,
      "inState": 71270235805491939,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2SetWithSpecifyingTypeTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2489491116942426158,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 73382690912632924,
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
      "id": 3737969493616125732,
      "inState": 4042038736016474424,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3534693618802586510,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1853996312728269919,
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
      "id": 3923063415261756327,
      "inState": 2489491116942426158,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3534693618802586510,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3053136108186850743,
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
      "id": 3030470219591531308,
      "inState": 71270235805491939,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2OverwriteValueWithDifferentTypeTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 454026604792850007,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3777408508957011885,
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
      "id": 3285909339681176986,
      "inState": 71270235805491939,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseTestState2AccessAsAnyTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4042038736016474424,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 572259393514094327,
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
