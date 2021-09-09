{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1614963977287,
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
  "masterPlan": false,
  "name": "SchedulingTestSequencePlan1",
  "preCondition": null,
  "relativeDirectory": "",
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
        "x": 428,
        "y": 200
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
        "x": 944,
        "y": 200
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
      "id": 1614964566530,
      "inState": 1614963979424,
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
      "id": 1614964572494,
      "inState": 1614964540694,
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
      "id": 1614964575552,
      "inState": 1614964541828,
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
      "id": 1614964578015,
      "inState": 1614964542678,
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