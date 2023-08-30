{
  "blackboard": [
    {
      "access": "input",
      "comment": "",
      "id": 776883745699711074,
      "key": "boolValueA",
      "type": "bool"
    },
    {
      "access": "input",
      "comment": "",
      "id": 1045429957834828049,
      "key": "doubleValueA",
      "type": "double"
    },
    {
      "access": "input",
      "comment": "",
      "id": 2663555862223669320,
      "key": "intValueA",
      "type": "int64"
    },
    {
      "access": "input",
      "comment": "",
      "id": 3117720767320173565,
      "key": "stringValueA",
      "type": "std::string"
    },
    {
      "access": "input",
      "comment": "",
      "id": 2746243078063830243,
      "key": "uintValueA",
      "type": "uint64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 31391243837222744,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2157089947574819410,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 750727936829194403,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 2157089947574819410,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "InheritFromParentTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 63072614606188547,
      "inTransitions": [
        1583385661528817487
      ],
      "name": "CheckStringState",
      "outTransitions": [
        667522708593373227
      ],
      "parentPlan": 2157089947574819410,
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
      "entryPoint": null,
      "id": 2595473976789985278,
      "inTransitions": [
        1032498883150016692
      ],
      "name": "CheckIntState",
      "outTransitions": [
        1583385661528817487
      ],
      "parentPlan": 2157089947574819410,
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
      "id": 2255632851020707596,
      "inTransitions": [
        3861936007093432974
      ],
      "name": "CheckDoubleState",
      "outTransitions": [
        1032498883150016692
      ],
      "parentPlan": 2157089947574819410,
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
      "id": 4527972571656844729,
      "inTransitions": [
        4173661187628445745
      ],
      "name": "CheckBoolState",
      "outTransitions": [
        3861936007093432974
      ],
      "parentPlan": 2157089947574819410,
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
      "id": 3866456247835935725,
      "inTransitions": [
        667522708593373227
      ],
      "name": "CheckUIntState",
      "outTransitions": [
        3032574709008121663
      ],
      "parentPlan": 2157089947574819410,
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
      "id": 3596014439409661935,
      "inTransitions": [
        3032574709008121663
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 2157089947574819410,
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
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "InheritFromParentATestPlan.pml#185950610054102790",
          "comment": "",
          "configuration": null,
          "id": 963463330093943381,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 31391243837222744,
      "id": 750727936829194403,
      "inTransitions": [],
      "name": "RunChildState",
      "outTransitions": [
        4173661187628445745
      ],
      "parentPlan": 2157089947574819410,
      "positionWeb": {
        "x": 428,
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
      "condition": "ConditionRepository.cnd#1",
      "id": 4173661187628445745,
      "inState": 750727936829194403,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4527972571656844729,
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
        "id": 1280267743044087321,
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
      "id": 667522708593373227,
      "inState": 63072614606188547,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "stringValueA",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": "setString"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3866456247835935725,
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
        "id": 2197652967974413170,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#89453017592951088",
      "id": 3861936007093432974,
      "inState": 4527972571656844729,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": null,
            "value": true
          },
          {
            "childKey": "left",
            "parentKey": "boolValueA",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2255632851020707596,
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
        "id": 2072312308530029012,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1644313031322732060",
      "id": 1032498883150016692,
      "inState": 2255632851020707596,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "doubleValueA",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 2.0
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2595473976789985278,
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
        "id": 4142442813779815446,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#344595839150634454",
      "id": 3032574709008121663,
      "inState": 3866456247835935725,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "uintValueA",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 2
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3596014439409661935,
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
        "id": 392334397707378843,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2618711924921632178",
      "id": 1583385661528817487,
      "inState": 2595473976789985278,
      "keyMapping": {
        "input": [
          {
            "childKey": "right",
            "parentKey": "intValueA",
            "value": null
          },
          {
            "childKey": "left",
            "parentKey": null,
            "value": 2
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 63072614606188547,
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
        "id": 272091851166634952,
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
