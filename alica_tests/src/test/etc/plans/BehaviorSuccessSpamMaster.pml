{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1522377375150,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1522377375148,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1522377375149,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1522377375148,
  "masterPlan": true,
  "name": "BehaviorSuccessSpamMaster",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/SuccessSpam.beh#1522377401286",
          "comment": "",
          "configuration": null,
          "id": 1587718663016,
          "name": "1587718663016"
        }
      ],
      "entryPoint": 1522377375150,
      "id": 1522377375149,
      "inTransitions": [
        1522377945069
      ],
      "name": "Normal",
      "outTransitions": [
        1522377944058
      ],
      "parentPlan": 1522377375148,
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
          "abstractPlan": "behaviours/SuccessSpam.beh#1522377401286",
          "comment": "",
          "configuration": null,
          "id": 1587718663020,
          "name": "1587718663020"
        }
      ],
      "entryPoint": null,
      "id": 1522377929290,
      "inTransitions": [
        1522377944058
      ],
      "name": "Dummy",
      "outTransitions": [
        1522377945069
      ],
      "parentPlan": 1522377375148,
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
      "id": 1522377944058,
      "inState": 1522377375149,
      "name": "MISSING_NAME",
      "outState": 1522377929290,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1522377944921,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 1522377945069,
      "inState": 1522377929290,
      "name": "MISSING_NAME",
      "outState": 1522377375149,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1522377946607,
        "name": "MISSING_NAME",
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