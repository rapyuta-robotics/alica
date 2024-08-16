{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 768854120689557866,
      "key": "CompareCondition2CompareLessThanString",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 773855036706857873,
      "key": "valueBool",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1323131406628204752,
      "key": "valueUint64",
      "type": "uint64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1455619520226910856,
      "key": "valueInt64",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1594166873455497029,
      "key": "valueDouble",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3758731991465578044,
      "key": "CompareCondition2CompareEqualString",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 4339086657243043033,
      "key": "valueString",
      "type": "std::string"
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
        "x": 200,
        "y": 412
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
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": true,
  "name": "StandardLibraryCompareConditionsPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 256024564977828539,
      "inTransitions": [
        1375797524455429289,
        4138163603892705882
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 2233.875,
        "y": 400
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 2524755550241225566,
      "id": 476060067190058583,
      "inTransitions": [],
      "name": "CompareCondition",
      "outTransitions": [
        728054401006425034,
        832466237305523478
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 427.984375,
        "y": 400
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
      "name": "CompareEqualString",
      "outTransitions": [
        4187059211928657606
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 685.96875,
        "y": 200
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
        757718038326292563
      ],
      "name": "CompareEqualInt64",
      "outTransitions": [
        1965826180516246479
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 1201.9375,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1413282794207559561,
      "inTransitions": [
        4526303688343435254
      ],
      "name": "CompareLessThanDone",
      "outTransitions": [
        4138163603892705882
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 1975.890625,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1469455919857934740,
      "inTransitions": [
        832466237305523478
      ],
      "name": "CompareLessThanString",
      "outTransitions": [
        970346257648053593
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 943.953125,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1763827601312058421,
      "inTransitions": [
        2967130899710135635
      ],
      "name": "CompareEqualBool",
      "outTransitions": [
        350880109410447955
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 1717.90625,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1895125957728601861,
      "inTransitions": [
        970346257648053593
      ],
      "name": "CompareLessThanDouble",
      "outTransitions": [
        3505302734271536465
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 1201.9375,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 2140695774352390511,
      "inTransitions": [
        350880109410447955
      ],
      "name": "CompareEqualDone",
      "outTransitions": [
        1375797524455429289
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 1975.890625,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 2261594950662502808,
      "inTransitions": [
        2702770775246679807
      ],
      "name": "CompareLessThanUInt64",
      "outTransitions": [
        4526303688343435254
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 1717.90625,
        "y": 400
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
      "name": "CompareEqualDouble",
      "outTransitions": [
        757718038326292563
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 943.953125,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3346970593385922729,
      "inTransitions": [
        1965826180516246479
      ],
      "name": "CompareEqualUInt64",
      "outTransitions": [
        2967130899710135635
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 1459.921875,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3447733804217434280,
      "inTransitions": [
        3505302734271536465
      ],
      "name": "CompareLessThanInt64",
      "outTransitions": [
        2702770775246679807
      ],
      "parentPlan": 570949913365648849,
      "positionWeb": {
        "x": 1459.921875,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#89453017592951088",
      "id": 350880109410447955,
      "inState": 1763827601312058421,
      "keyMapping": {
        "input": [
          {
            "childKey": "left",
            "parentKey": "valueBool",
            "value": null
          },
          {
            "childKey": "right",
            "parentKey": "valueBool",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2140695774352390511,
      "pointsWeb": [
        {
          "x": 1875.890625,
          "y": 228.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2924642076521695534,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 728054401006425034,
      "inState": 476060067190058583,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "CompareCondition2CompareEqualString",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 622940645707226126,
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
        "id": 1047898305021363246,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1644313031322732060",
      "id": 757718038326292563,
      "inState": 2326317223569681786,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "valueDouble",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 3.15
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 717240492377837483,
      "pointsWeb": [
        {
          "x": 1101.9375,
          "y": 228.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 855000311320764054,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 832466237305523478,
      "inState": 476060067190058583,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "CompareCondition2CompareLessThanString",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1469455919857934740,
      "pointsWeb": [
        {
          "x": 585.96875,
          "y": 428.9921875
        },
        {
          "x": 714.9609375,
          "y": 428.9921875
        },
        {
          "x": 843.953125,
          "y": 428.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 852000274527975769,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1496803185713136824",
      "id": 970346257648053593,
      "inState": 1469455919857934740,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "valueString",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": "abc"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1895125957728601861,
      "pointsWeb": [
        {
          "x": 1101.9375,
          "y": 428.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1224247015051878961,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2872265442510628524",
      "id": 1375797524455429289,
      "inState": 2140695774352390511,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 256024564977828539,
      "pointsWeb": [
        {
          "x": 2133.875,
          "y": 228.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3347053728260651190,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 1965826180516246479,
      "inState": 717240492377837483,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "valueInt64",
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
      "pointsWeb": [
        {
          "x": 1359.921875,
          "y": 228.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1213722519075398657,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#997531702947519653",
      "id": 2702770775246679807,
      "inState": 3447733804217434280,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "valueInt64",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": -1234
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2261594950662502808,
      "pointsWeb": [
        {
          "x": 1617.90625,
          "y": 428.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1757990176379053066,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#344595839150634454",
      "id": 2967130899710135635,
      "inState": 3346970593385922729,
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
      "outState": 1763827601312058421,
      "pointsWeb": [
        {
          "x": 1617.90625,
          "y": 228.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4164263715291503935,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1704333014293218519",
      "id": 3505302734271536465,
      "inState": 1895125957728601861,
      "keyMapping": {
        "input": [
          {
            "childKey": "left",
            "parentKey": null,
            "value": 3.13
          },
          {
            "childKey": "right",
            "parentKey": "valueDouble",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3447733804217434280,
      "pointsWeb": [
        {
          "x": 1359.921875,
          "y": 428.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2960597575116124097,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2872265442510628524",
      "id": 4138163603892705882,
      "inState": 1413282794207559561,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 256024564977828539,
      "pointsWeb": [
        {
          "x": 2133.875,
          "y": 428.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4392364796298833008,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#4434919629521912839",
      "id": 4187059211928657606,
      "inState": 622940645707226126,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "valueString",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": "xyz"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2326317223569681786,
      "pointsWeb": [
        {
          "x": 843.953125,
          "y": 228.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3182085130989509059,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3240875604708587637",
      "id": 4526303688343435254,
      "inState": 2261594950662502808,
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
            "value": 1
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1413282794207559561,
      "pointsWeb": [
        {
          "x": 1875.890625,
          "y": 428.9921875
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3256088237934994159,
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
