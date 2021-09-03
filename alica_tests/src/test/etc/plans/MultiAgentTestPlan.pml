{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1413200877337,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "AttackTask",
      "plan": 1413200862180,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1413200877336,
      "successRequired": true,
      "task": "taskrepository.tsk#1407153522080"
    },
    {
      "comment": "",
      "id": 1413200890537,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "DefaultTask",
      "plan": 1413200862180,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1413200910490,
      "successRequired": true,
      "task": "taskrepository.tsk#1225112227903"
    },
    {
      "comment": "",
      "id": 1413807260446,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "NewEntryPoint",
      "plan": 1413200862180,
      "positionWeb": {
        "x": 200,
        "y": 612
      },
      "state": 1413807264574,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1413200862180,
  "masterPlan": false,
  "name": "MultiAgentTestPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/Attack.beh#1402488848841",
          "comment": "",
          "configuration": null,
          "id": 1587718662849,
          "name": "1587718662849"
        }
      ],
      "entryPoint": 1413200877337,
      "id": 1413200877336,
      "inTransitions": [],
      "name": "OtherState",
      "outTransitions": [
        1413201368286
      ],
      "parentPlan": 1413200862180,
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
          "abstractPlan": "behaviours/Attack.beh#1402488848841",
          "comment": "",
          "configuration": null,
          "id": 1587718662852,
          "name": "1587718662852"
        }
      ],
      "entryPoint": 1413200890537,
      "id": 1413200910490,
      "inTransitions": [],
      "name": "State1",
      "outTransitions": [
        1413201050743
      ],
      "parentPlan": 1413200862180,
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
          "abstractPlan": "behaviours/Attack.beh#1402488848841",
          "comment": "",
          "configuration": null,
          "id": 1587718662855,
          "name": "1587718662855"
        }
      ],
      "entryPoint": null,
      "id": 1413201030936,
      "inTransitions": [
        1413201050743
      ],
      "name": "State2",
      "outTransitions": [
        1413201367062
      ],
      "parentPlan": 1413200862180,
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
      "id": 1413201164999,
      "inTransitions": [
        1413201368286
      ],
      "name": "NewSuccessState1",
      "outTransitions": [],
      "parentPlan": 1413200862180,
      "positionWeb": {
        "x": 686,
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
      "entryPoint": null,
      "id": 1413552736921,
      "inTransitions": [
        1413201367062
      ],
      "name": "NewSuccessState2",
      "outTransitions": [],
      "parentPlan": 1413200862180,
      "positionWeb": {
        "x": 944,
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
          "abstractPlan": "behaviours/AttackOpp.beh#1402489351885",
          "comment": "",
          "configuration": null,
          "id": 1587718662860,
          "name": "1587718662860"
        }
      ],
      "entryPoint": 1413807260446,
      "id": 1413807264574,
      "inTransitions": [],
      "name": "Idle",
      "outTransitions": [],
      "parentPlan": 1413200862180,
      "positionWeb": {
        "x": 428,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "id": 1413201050743,
      "inState": 1413200910490,
      "name": "1413201050743",
      "outState": 1413201030936,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1413201052549,
        "name": "1413201052549",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 1413201367062,
      "inState": 1413201030936,
      "name": "1413201367062",
      "outState": 1413552736921,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1413201367990,
        "name": "1413201367990",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 1413201368286,
      "inState": 1413200877336,
      "name": "1413201368286",
      "outState": 1413201164999,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1413201370590,
        "name": "1413201370590",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": []
}