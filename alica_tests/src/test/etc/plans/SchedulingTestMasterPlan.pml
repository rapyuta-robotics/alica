{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1615797283419,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1615797283419",
      "plan": 1613378382024,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1615797271229,
      "successRequired": false,
      "task": "taskrepository.tsk#1613371619454"
    }
  ],
  "frequency": 0,
  "id": 1613378382024,
  "masterPlan": true,
  "name": "SchedulingTestMasterPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "requiresParameters": false,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SchedulingTestPlan1.pml#1613378406860",
          "comment": "",
          "configuration": null,
          "id": 1613378494690,
          "name": "1613378494690"
        }
      ],
      "entryPoint": null,
      "id": 1613378474109,
      "inTransitions": [
        1615797316170
      ],
      "name": "InitTest",
      "outTransitions": [
        1613530643879
      ],
      "parentPlan": 1613378382024,
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
      "id": 1613530614559,
      "inTransitions": [
        1613530643879,
        1615797365363,
        1629895607017
      ],
      "name": "EndTest",
      "outTransitions": [],
      "parentPlan": 1613378382024,
      "positionWeb": {
        "x": 944,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1615797283419,
      "id": 1615797271229,
      "inTransitions": [],
      "name": "Default Name",
      "outTransitions": [
        1615797316170,
        1615797327076,
        1629895598464
      ],
      "parentPlan": 1613378382024,
      "positionWeb": {
        "x": 428,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SchedulingTestSequencePlan1.pml#1614963946725",
          "comment": "",
          "configuration": null,
          "id": 1615797379906,
          "name": "1615797379906"
        }
      ],
      "entryPoint": null,
      "id": 1615797319003,
      "inTransitions": [
        1615797327076
      ],
      "name": "Default Name",
      "outTransitions": [
        1615797365363
      ],
      "parentPlan": 1613378382024,
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
          "abstractPlan": "OrderedSchedulingTestPlan.pml#1629895582410",
          "comment": "",
          "configuration": null,
          "id": 1629895663156,
          "name": "1629895663156"
        }
      ],
      "entryPoint": null,
      "id": 1629895593451,
      "inTransitions": [
        1629895598464
      ],
      "name": "OrderedSchedulingTestPlan",
      "outTransitions": [
        1629895607017
      ],
      "parentPlan": 1613378382024,
      "positionWeb": {
        "x": 686,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "MISSING_COMMENT",
      "id": 1613530643879,
      "inState": 1613378474109,
      "name": "FromDefault NameTo EndTest",
      "outState": 1613530614559,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1613530643882,
        "name": "1613530643882",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "id": 1615797316170,
      "inState": 1615797271229,
      "name": "FromDefault NameTo InitTest",
      "outState": 1613378474109,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1615797316171,
        "name": "1615797316171",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "id": 1615797327076,
      "inState": 1615797271229,
      "name": "FromDefault NameTo Default Name",
      "outState": 1615797319003,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1615797327077,
        "name": "1615797327077",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "id": 1615797365363,
      "inState": 1615797319003,
      "name": "FromDefault NameTo EndTest",
      "outState": 1613530614559,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1615797365364,
        "name": "1615797365364",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "id": 1629895598464,
      "inState": 1615797271229,
      "name": "FromDefault NameTo Default Name",
      "outState": 1629895593451,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1629895598471,
        "name": "1629895598471",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "id": 1629895607017,
      "inState": 1629895593451,
      "name": "FromDefault NameTo EndTest",
      "outState": 1613530614559,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1629895607018,
        "name": "1629895607018",
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