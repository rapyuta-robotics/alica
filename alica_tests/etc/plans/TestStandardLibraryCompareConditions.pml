{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 1455619520226910856,
      "key": "valueInt64",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1594166873455497029,
      "key": "valueDouble",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 4339086657243043033,
      "key": "valueString",
      "type": "std::string"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1323131406628204752,
      "key": "valueUint64",
      "type": "uint64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2524755550241225566,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 570949913365648849,
      "positionWeb": {
        "x": 40.20892807408327,
        "y": 375.4483458218895
      },
      "state": 476060067190058583,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 570949913365648849,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": true,
  "name": "TestStandardLibraryCompareConditions",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3346970593385922729,
      "inTransitions": [
        1965826180516246479
      ],
      "name": "DoneCompare",
      "outTransitions": [],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 875.856183862036,
        "y": 363.914506705205
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 717240492377837483,
      "inTransitions": [
        1676173970773717201
      ],
      "name": "CompareGreateThanEqual",
      "outTransitions": [
        1965826180516246479
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 719.6056646889159,
        "y": 165.03500215095843
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 2524755550241225566,
      "id": 476060067190058583,
      "inTransitions": [],
      "name": "CompareEqual",
      "outTransitions": [
        728054401006425034
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 187.99580730260888,
        "y": 366.40439203633144
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 622940645707226126,
      "inTransitions": [
        728054401006425034
      ],
      "name": "CompareNotEqual",
      "outTransitions": [
        4187059211928657606
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 352.1590787454322,
        "y": 165.16526206465124
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 2326317223569681786,
      "inTransitions": [
        4187059211928657606
      ],
      "name": "CompareLessThan",
      "outTransitions": [
        1676173970773717201
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 527,
        "y": 362.89488983154297
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2056838422073723411",
      "id": 4187059211928657606,
      "inState": 622940645707226126,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": null,
            "value": 3.14
          },
          {
            "childKey": "left",
            "parentKey": "valueDouble",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2326317223569681786,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3182085130989509059,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#4434919629521912839",
      "id": 728054401006425034,
      "inState": 476060067190058583,
      "keyMapping": {
        "input": [
          {
            "childKey": "left",
            "parentKey": null,
            "value": "test_equal_string"
          },
          {
            "childKey": "right",
            "parentKey": "valueString",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 622940645707226126,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1047898305021363246,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#997531702947519653",
      "id": 1676173970773717201,
      "inState": 2326317223569681786,
      "keyMapping": {
        "input": [
          {
            "childKey": "left",
            "parentKey": "valueInt64",
            "value": null
          },
          {
            "childKey": "right",
            "parentKey": null,
            "value": -4567
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 717240492377837483,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4388657879949165251,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#718333707505171884",
      "id": 1965826180516246479,
      "inState": 717240492377837483,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "valueUint64",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 1234
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3346970593385922729,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1213722519075398657,
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
