{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1604125167842760918,
      "key": "ChooseBlackboardTestState2ValueMappingPlanTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 2499503172139630260,
      "key": "ChooseBlackboardTestState2ValueMappingBehaviourTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3790159344295070800,
      "key": "ChooseBlackboardTestState2ValueMappingConditionTestState",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3928341634460937152,
      "key": "ChooseBlackboardTestState2PopulateBlackboardTestState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1656223453390456865,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1633245244310547016,
      "positionWeb": {
        "x": 200,
        "y": 612
      },
      "state": 950259230717695500,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1633245244310547016,
  "implementationName": "",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "BlackboardTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "ValueMappingTestBeh.beh#4427168067193548436",
          "comment": "",
          "configuration": null,
          "id": 411097553873889481,
          "keyMapping": {
            "input": [
              {
                "childKey": "mappedBoolValue",
                "parentKey": null,
                "value": true
              },
              {
                "childKey": "mappedStringValue",
                "parentKey": null,
                "value": "test"
              },
              {
                "childKey": "mappedIntValue",
                "parentKey": null,
                "value": -9
              },
              {
                "childKey": "mappedUintValue",
                "parentKey": null,
                "value": 5
              },
              {
                "childKey": "mappedDoubleValue",
                "parentKey": null,
                "value": 4.5
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 261975287879892078,
      "inTransitions": [
        2898451417708501510
      ],
      "name": "ValueMappingBehaviourTestState",
      "outTransitions": [
        2617458369167221176
      ],
      "parentPlan": 1633245244310547016,
      "positionWeb": {
        "x": 686,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1656223453390456865,
      "id": 950259230717695500,
      "inTransitions": [],
      "name": "ChooseBlackboardTestState",
      "outTransitions": [
        1591792576101512594,
        2025444659250378559,
        2898451417708501510,
        3982256618930995182
      ],
      "parentPlan": 1633245244310547016,
      "positionWeb": {
        "x": 428,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "ValueMappingPlanTestPlan.pml#3938799313883693450",
          "comment": "",
          "configuration": null,
          "id": 1419163199666490998,
          "keyMapping": {
            "input": [
              {
                "childKey": "mappedUintValue",
                "parentKey": null,
                "value": 19
              },
              {
                "childKey": "mappedStringValue",
                "parentKey": null,
                "value": "test"
              },
              {
                "childKey": "mappedDoubleValue",
                "parentKey": null,
                "value": 12.6
              },
              {
                "childKey": "mappedBoolValue",
                "parentKey": null,
                "value": true
              },
              {
                "childKey": "mappedIntValue",
                "parentKey": null,
                "value": -8
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1201416164340772348,
      "inTransitions": [
        1591792576101512594
      ],
      "name": "ValueMappingPlanTestState",
      "outTransitions": [
        543853498590134994
      ],
      "parentPlan": 1633245244310547016,
      "positionWeb": {
        "x": 686,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PopulateBlackboardTestPlan.pml#2264579918884483840",
          "comment": "",
          "configuration": null,
          "id": 1616198078937108437,
          "keyMapping": {
            "input": [
              {
                "childKey": "data",
                "parentKey": null,
                "value": "{\"bool_key\": true, \"int64_key\": -111, \"uint64_key\": 222, \"double_key\": 3.33, \"string_key\": \"some_value\", \"not_in_bb\": 1}"
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1363957399826863176,
      "inTransitions": [
        2025444659250378559
      ],
      "name": "PopulateBlackboardTestState",
      "outTransitions": [
        3610708172177662828
      ],
      "parentPlan": 1633245244310547016,
      "positionWeb": {
        "x": 686,
        "y": 800
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3953151813073280186,
      "inTransitions": [
        171823477339840252,
        543853498590134994,
        2617458369167221176,
        3610708172177662828
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 1633245244310547016,
      "positionWeb": {
        "x": 944,
        "y": 800
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
      "id": 4410686024647208677,
      "inTransitions": [
        3982256618930995182
      ],
      "name": "ValueMappingConditionTestState",
      "outTransitions": [
        171823477339840252
      ],
      "parentPlan": 1633245244310547016,
      "positionWeb": {
        "x": 686,
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
      "condition": "ConditionRepository.cnd#2046573221100914383",
      "id": 171823477339840252,
      "inState": 4410686024647208677,
      "keyMapping": {
        "input": [
          {
            "childKey": "mappedUintValue",
            "parentKey": null,
            "value": 17
          },
          {
            "childKey": "mappedIntValue",
            "parentKey": null,
            "value": -3
          },
          {
            "childKey": "mappedBoolValue",
            "parentKey": null,
            "value": true
          },
          {
            "childKey": "mappedStringValue",
            "parentKey": null,
            "value": "test"
          },
          {
            "childKey": "mappedDoubleValue",
            "parentKey": null,
            "value": 3.7
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3953151813073280186,
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
        "id": 535013460795749320,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1552609259127937138",
      "id": 543853498590134994,
      "inState": 1201416164340772348,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3953151813073280186,
      "pointsWeb": [
        {
          "x": 844,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4508134782915683308,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 1591792576101512594,
      "inState": 950259230717695500,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseBlackboardTestState2ValueMappingPlanTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1201416164340772348,
      "pointsWeb": [
        {
          "x": 586,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4295754783734115472,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 2025444659250378559,
      "inState": 950259230717695500,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseBlackboardTestState2PopulateBlackboardTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 1363957399826863176,
      "pointsWeb": [
        {
          "x": 586,
          "y": 829
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1839484836184675310,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1552609259127937138",
      "id": 2617458369167221176,
      "inState": 261975287879892078,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3953151813073280186,
      "pointsWeb": [
        {
          "x": 844,
          "y": 629
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 87492245890234575,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 2898451417708501510,
      "inState": 950259230717695500,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseBlackboardTestState2ValueMappingBehaviourTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 261975287879892078,
      "pointsWeb": [
        {
          "x": 586,
          "y": 629
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1720760597091928932,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 3610708172177662828,
      "inState": 1363957399826863176,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3953151813073280186,
      "pointsWeb": [
        {
          "x": 844,
          "y": 829
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3708663340461492332,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 3982256618930995182,
      "inState": 950259230717695500,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "ChooseBlackboardTestState2ValueMappingConditionTestState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4410686024647208677,
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
        "id": 124474551502599750,
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
