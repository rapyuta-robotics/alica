{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 14128325898465596,
      "key": "not_in_data_key",
      "type": "std::string"
    },
    {
      "access": "input",
      "comment": "",
      "defaultValue": null,
      "id": 37366997480187649,
      "key": "data",
      "type": "std::string"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1830052697140273434,
      "key": "double_key",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3065125271564665902,
      "key": "bool_key",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3141833006463785136,
      "key": "OtherChecksState2PopulatedState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3179113368759389835,
      "key": "int64_key",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3542073533896409826,
      "key": "string_key",
      "type": "std::string"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 4501786985394765844,
      "key": "uint64_key",
      "type": "uint64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2678296145075772286,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2264579918884483840,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 2156041826637853351,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 2264579918884483840,
  "implementationName": "PopulateBlackboard",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "PopulateBlackboardTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 269468753681610364,
      "inTransitions": [
        4564317409522652618
      ],
      "name": "PopulatedState",
      "outTransitions": [],
      "parentPlan": 2264579918884483840,
      "positionWeb": {
        "x": 1976,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 725628133109138265,
      "inTransitions": [
        1693841970976456590
      ],
      "name": "DoubleCheckState",
      "outTransitions": [
        2264383531314509842
      ],
      "parentPlan": 2264579918884483840,
      "positionWeb": {
        "x": 1202,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 929284348432123043,
      "inTransitions": [
        3189853583143607438
      ],
      "name": "IntCheckState",
      "outTransitions": [
        2870251049497860317
      ],
      "parentPlan": 2264579918884483840,
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
      "id": 1543689537208309158,
      "inTransitions": [
        2264383531314509842
      ],
      "name": "StringCheckState",
      "outTransitions": [
        12809725883667270
      ],
      "parentPlan": 2264579918884483840,
      "positionWeb": {
        "x": 1460,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 2678296145075772286,
      "id": 2156041826637853351,
      "inTransitions": [],
      "name": "BoolCheckState",
      "outTransitions": [
        3189853583143607438
      ],
      "parentPlan": 2264579918884483840,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 2489853994917446865,
      "inTransitions": [
        12809725883667270
      ],
      "name": "OtherChecksState",
      "outTransitions": [
        4564317409522652618
      ],
      "parentPlan": 2264579918884483840,
      "positionWeb": {
        "x": 1718,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 2998742479977225081,
      "inTransitions": [
        2870251049497860317
      ],
      "name": "UintCheckState",
      "outTransitions": [
        1693841970976456590
      ],
      "parentPlan": 2264579918884483840,
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
      "condition": "ConditionRepository.cnd#4434919629521912839",
      "id": 12809725883667270,
      "inState": 1543689537208309158,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": null,
            "value": "some_value"
          },
          {
            "childKey": "left",
            "parentKey": "string_key",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2489853994917446865,
      "pointsWeb": [
        {
          "x": 1618,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3983408236047742234,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#344595839150634454",
      "id": 1693841970976456590,
      "inState": 2998742479977225081,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "uint64_key",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 222
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 725628133109138265,
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
        "id": 1039582954287951959,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1644313031322732060",
      "id": 2264383531314509842,
      "inState": 725628133109138265,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "double_key",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 3.33
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1543689537208309158,
      "pointsWeb": [
        {
          "x": 1360,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4163240752266831361,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 2870251049497860317,
      "inState": 929284348432123043,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "int64_key",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": -111
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2998742479977225081,
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
        "id": 4328557078564248177,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#89453017592951088",
      "id": 3189853583143607438,
      "inState": 2156041826637853351,
      "keyMapping": {
        "input": [
          {
            "childKey": "left",
            "parentKey": "bool_key",
            "value": null
          },
          {
            "childKey": "right",
            "parentKey": null,
            "value": true
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 929284348432123043,
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
        "id": 4158752398488433093,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 4564317409522652618,
      "inState": 2489853994917446865,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "OtherChecksState2PopulatedState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 269468753681610364,
      "pointsWeb": [
        {
          "x": 1876,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1905505569871034119,
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
