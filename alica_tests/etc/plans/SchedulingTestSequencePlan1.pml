{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 620992452739501207,
      "key": "SubOne2SubTwo",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1998335174749558241,
      "key": "Sequenze2SubPlan",
      "type": "int64"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3390934806840785672,
      "key": "InitSubThree2TermSubThree",
      "type": "int64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1614963977287,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1614963977287",
      "plan": 1614963946725,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1614963979424,
      "successRequired": false,
      "task": "taskrepository.tsk#1613372009777"
    }
  ],
  "frequency": 0,
  "id": 1614963946725,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "SchedulingTestSequencePlan1",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1614963977287,
      "id": 1614963979424,
      "inTransitions": [],
      "name": "InitSequencePlan1",
      "outTransitions": [
        1614964566530
      ],
      "parentPlan": 1614963946725,
      "positionWeb": {
        "x": 338.1214848905282,
        "y": 4.795099996616045
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SchedulingTestSequenceSubPlan1.pml#1614964379654",
          "comment": "",
          "configuration": null,
          "id": 1614964583934,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1614964583934"
        }
      ],
      "entryPoint": null,
      "id": 1614964540694,
      "inTransitions": [
        1614964566530
      ],
      "name": "InitSequenceSubPlan1",
      "outTransitions": [
        1614964572494
      ],
      "parentPlan": 1614963946725,
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
          "abstractPlan": "SchedulingTestSequenceSubPlan2.pml#1614964444419",
          "comment": "",
          "configuration": null,
          "id": 1614964588445,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1614964588445"
        }
      ],
      "entryPoint": null,
      "id": 1614964541828,
      "inTransitions": [
        1614964572494
      ],
      "name": "InitSequenceSubPlan2",
      "outTransitions": [
        1614964575552
      ],
      "parentPlan": 1614963946725,
      "positionWeb": {
        "x": 944.0463456899656,
        "y": -16.902834346510843
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SchedulingTestSequenceSubPlan3.pml#1614964478264",
          "comment": "",
          "configuration": null,
          "id": 1614964591096,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1614964591096"
        }
      ],
      "entryPoint": null,
      "id": 1614964542678,
      "inTransitions": [
        1614964575552
      ],
      "name": "InitSequenceSubPlan3",
      "outTransitions": [
        1614964578015
      ],
      "parentPlan": 1614963946725,
      "positionWeb": {
        "x": 1305.9220330953267,
        "y": -6.439714392067941
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1614964543300,
      "inTransitions": [
        1614964578015
      ],
      "name": "TerminateSequenceSubPlan3",
      "outTransitions": [],
      "parentPlan": 1614963946725,
      "positionWeb": {
        "x": 1460,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "MISSING_COMMENT",
      "condition": "ConditionRepository.cnd#2901825906319407673",
      "id": 1614964566530,
      "inState": 1614963979424,
      "keyMapping": {
        "input": [
          {
            "childKey": "numberOfCalls",
            "parentKey": "Sequenze2SubPlan",
            "value": null
          }
        ],
        "output": []
      },
      "name": "FromInitSequencePlan1To InitSequenceSubPlan1",
      "outState": 1614964540694,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1614964566531,
        "name": "1614964566531",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "condition": "ConditionRepository.cnd#2901825906319407673",
      "id": 1614964572494,
      "inState": 1614964540694,
      "keyMapping": {
        "input": [
          {
            "childKey": "numberOfCalls",
            "parentKey": "SubOne2SubTwo",
            "value": null
          }
        ],
        "output": []
      },
      "name": "FromInitSequenceSubPlan1To InitSequenceSubPlan2",
      "outState": 1614964541828,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1614964572495,
        "name": "1614964572495",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "condition": "ConditionRepository.cnd#2872265442510628524",
      "id": 1614964575552,
      "inState": 1614964541828,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "FromInitSequenceSubPlan2To InitSequenceSubPlan3",
      "outState": 1614964542678,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1614964575553,
        "name": "1614964575553",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "condition": "ConditionRepository.cnd#2901825906319407673",
      "id": 1614964578015,
      "inState": 1614964542678,
      "keyMapping": {
        "input": [
          {
            "childKey": "numberOfCalls",
            "parentKey": "InitSubThree2TermSubThree",
            "value": null
          }
        ],
        "output": []
      },
      "name": "FromInitSequenceSubPlan3To TerminateSequenceSubPlan3",
      "outState": 1614964543300,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1614964578016,
        "name": "1614964578016",
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
